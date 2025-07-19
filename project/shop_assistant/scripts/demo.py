#!/usr/bin/env python3
import rospy
from std_msgs.msg import String, Empty, Bool, Int32
from geometry_msgs.msg import Twist
import requests
import json
import time
import random
from rdflib import Graph, Namespace, Literal, RDF, RDFS, URIRef


class ShopOntologyManager:
    def __init__(self, ontology_path):
        self.graph = Graph()
        self.ontology_path = ontology_path
        self.graph.parse(ontology_path, format='ttl')
        self.SHOP = Namespace("http://example.org/shop#")
        self.graph.bind("shop", self.SHOP)

    def save(self):
        self.graph.serialize(destination=self.ontology_path, format='ttl')

    def create_user(self, user_name):
        user_uri = self.SHOP[user_name]
        self.graph.add((user_uri, RDF.type, self.SHOP.User))
        print(f"User '{user_name}' created.")

    def product_has_allergen(self, product_name, allergen_name):
        product_uri = self.SHOP[product_name]
        allergen_uri = self.SHOP[allergen_name]

        return (product_uri, self.SHOP.hasAllergen, allergen_uri) in self.graph
    
    def user_exists(self, user_name):
        user_uri = self.SHOP[user_name]
        return (user_uri, RDF.type, self.SHOP.User) in self.graph


    def create_user(self, user_name):
        user_uri = self.SHOP[user_name]
        self.graph.add((user_uri, RDF.type, self.SHOP.User))
        print(f"User '{user_name}' created.")
        self.save()

    def add_allergy_to_user(self, user_name, allergen_name):
        if not self.user_exists(user_name):
            self.create_user(user_name)
        user_uri = self.SHOP[user_name]
        allergen_uri = self.SHOP[allergen_name]
        self.graph.add((user_uri, self.SHOP.isAllergicTo, allergen_uri))
        print(f"Added allergy '{allergen_name}' to user '{user_name}'.")
        self.save()

    def get_product_location(self, product_name):
        product_uri = self.SHOP[product_name]
        q = """
        PREFIX shop: <http://example.org/shop#>
        SELECT ?location WHERE {
            shop:%s shop:locatedIn ?location .
        }
        """ % product_name

        results = self.graph.query(q)
        locations = [str(row.location).split("#")[-1] for row in results]
        if locations:
            return locations[0]
        else:
            return None

    def product_contains_alcohol(self, product_name):
        q = """
        PREFIX shop: <http://example.org/shop#>
        SELECT ?alcohol WHERE {
            shop:%s shop:containsAlcohol ?alcohol .
        }
        """ % product_name

        results = self.graph.query(q)
        for row in results:
            val = str(row.alcohol).lower()
            if val == "true":
                return True
            elif val == "false":
                return False
        return None
    
    def get_conflicting_allergens(self, product_name, user_name):
        query = f"""
        PREFIX shop: <http://example.org/shop#>

        SELECT ?allergen WHERE {{
            shop:{product_name} shop:hasAllergen ?allergen .
            shop:{user_name} shop:isAllergicTo ?allergen .
        }}
        """

        results = self.graph.query(query)
        for row in results:
            print(row)
        print(query, results)
        return [str(row.allergen).split("#")[-1] for row in results]


    
    def product_exists(self, product_name):
        product_uri = self.SHOP[product_name]
        return (product_uri, None, None) in self.graph


class State:
    def __init__(self, parent):
        self.parent = parent

    def process_speech(self, msg):
        raise NotImplementedError
    
    def tts_done(self):
        self.parent.trigger_stt()


class InteractionFinishedState(State):
    def __init__(self, parent, prefix=""):
        super().__init__(parent)
        parent.say(prefix + "Is there anything else I can help you with?")

    def process_speech(self, msg):
        prompt = f"""
            The robot just finished helping a customer and asked if the customer needs anything. The customer said: "{msg}"
            Do they need anymore help? Answer with "yes" or "no" only.
        """
        answer = self.parent.make_openai_request(prompt)
        if not answer is None and "yes" in answer:
            new_state = InitialState(self.parent)
            return new_state.process_speech(msg) 


class LeadingUserToPruductState(State):
    def __init__(self, parent, product):
        super().__init__(parent)
        self.product = product
        parent.say("Follow me")

    def process_speech(self, msg):
        return InteractionFinishedState(self.parent)
    
    def tts_done(self):
        self.parent.simulate_movement()


class DirectionsFeedbackState(State):
    def __init__(self, parent, product):
        super().__init__(parent)
        self.product = product
        location = self.parent.ontology.get_product_location(product)
        rospy.loginfo(f"Product is located in {location}")
        parent.say(f"You can find that in {location}, to your left. Do you think you can find that?")

    def process_speech(self, msg):
        prompt = f"""
            The assistant has given direction to a product to which the customer replied: "{msg}"
            Are they satisfied with the directions given by the assistant? Answer with "yes" or "no" only.
        """
        answer = self.parent.make_openai_request(prompt)
        if not answer is None and "yes" in answer:
            return InteractionFinishedState(self.parent) 
        return LeadingUserToPruductState(self.parent, self.product)


class AskForIdState(State):
    def __init__(self, parent, product):
        super().__init__(parent)
        self.product = product
        parent.say("Can you show me your ID please?")
    
    def process_speech(self, msg):
        prompt = f"""
        A customer is looking for an alcoholic product and the assistant has asked them for an ID. The customer responded with: "{msg}"
        Based on the response is the user showing an ID or not? Answer with "yes" or "no"
        """
        answer = self.parent.make_openai_request(prompt)
        if not answer is None and "yes" in answer:
            return DirectionsFeedbackState(self.parent, self.product)
        return InteractionFinishedState(self.parent, prefix="Without an ID I cannot help you buy alcoholic products.")


class ProductNotAvailableState(State):
    def __init__(self, parent, product):
        super().__init__(parent)
        parent.say(f"Currently we do not have any {product}, unfortunately")

    def speech_done(self):
        return InteractionFinishedState(self.parent)

class WarnAboutAllergyState(State):
    def __init__(self, parent, product, allergen):
        super().__init__(parent)
        self.product = product
        self.allergen = allergen
        parent.say(f"Sorry, just to double check! {product} contains {allergen}, are you not allergic to that?")

    def process_speech(self, msg):
        prompt = f"""
        A customer is looking for a product and the shop assistant has warned them that they might be allergic
        to that product given previous interactions. The user said this: "{msg}".
        Do they confirm their allergy? Answer with yes or no.
        """
        ans = self.parent.make_openai_request(prompt)
        if("yes" in ans):
            return InteractionFinishedState(self.parent, prefix="Sorry to hear that.")
        return DirectionsFeedbackState(self.parent, self.product)

class AskingForDirectionsState(State):
    def check_if_alcoholic(self, product):
        return product in ["beer", "wine", "vodka"]

    def process_speech(self, msg):
        prompt = f"""
        A customer is looking for directions to a product in the store and said this: "{msg}"
        What product are they looking for? Reply with only the name of the product
        """
        product = self.parent.make_openai_request(prompt)
        rospy.loginfo(f"User is looking for {product}")
        self.product = product
        if not self.parent.ontology.product_exists(product):
            rospy.loginfo(f"Product does not exist")
            return ProductNotAvailableState(self.parent, product)
            
        if self.parent.ontology.product_contains_alcohol(product):
            self.product = product
            rospy.loginfo("Alcoholic product detected, triggering age verification")
            if self.parent.age <= 18:
                return AskForIdState(self.parent, self.product)
            rospy.loginfo("User was determined to be above 18")
            return DirectionsFeedbackState(self.parent, self.product)
        allergens = self.parent.ontology.get_conflicting_allergens(self.product, self.parent.user_id)
        if len(allergens) == 0: 
            return DirectionsFeedbackState(self.parent, self.product)
        else:
            return WarnAboutAllergyState(self.parent, self.product, allergens[0])


class AskingForAllergensState(State):
    def __init__(self, parent):
        super().__init__(parent)

    def process_speech(self, msg):
        prompt = f"""
        A customer is looking for allergy information of a product at the store and said this: "{msg}"
        What product and what allergy are they looking for? Reply with the name of the product and the allergen separated with a comma.
        """
        answer = self.parent.make_openai_request(prompt)
        product, self.allergen = [s.strip() for s in answer.split(',')] 
        self.contains_allergy =  self.parent.ontology.product_has_allergen(product, self.allergen)
        self.parent.ontology.add_allergy_to_user(self.parent.user_id, self.allergen)
        return InteractionFinishedState(self.parent, prefix="Yes it does." if self.contains_allergy else "No it does not.")

class UnkownFlowState(State):
    def __init__(self, parent):
        super().__init__(parent)
        parent.say("Sorry I cannot help with that. Is there anything else you would want help with?")

    def process_speech(self, msg):
        new_state = InitialState(parent=self.parent)
        return new_state.process_speech(msg)


class InitialState(State):
    def __init__(self, parent, first_start=False):
        super().__init__(parent)
        if first_start:
            self.parent.trigger_age_verification()
            self.parent.trigger_customer_id()
            parent.say("How can I help you?")
    
    def process_speech(self, msg):
        prompt = f"""
        A customer said: "{msg}"
        What do they want? Reply with one of: "directions", "allergen_check", or "unknown".
        """
        answer = self.parent.make_openai_request(prompt)
        if answer is None:
            return self
        if "directions" in answer:
            new_state = AskingForDirectionsState(self.parent) 
            return new_state.process_speech(msg)
        if "allergen_check" in answer:
            new_state = AskingForAllergensState(self.parent)
            return new_state.process_speech(msg)
        return UnkownFlowState(self.parent)


class ShopAssistant:
    def __init__(self):
        rospy.init_node('shop_assistant_node')
        self.ontology = ShopOntologyManager("/home/tiago_public_ws/src/project/ontology/shop_demo.ttl")

        self.stt_pub = rospy.Publisher('/start_listening', Empty, queue_size=10)
        self.tts_pub = rospy.Publisher('/speak_text', String, queue_size=10)
        self.move_pub = rospy.Publisher('/mobile_base_controller/cmd_vel', Twist, queue_size=10)
        self.age_verification_pub = rospy.Publisher("/check_age", Bool, queue_size=10)
        self.customer_id_pub = rospy.Publisher("/customer_id", Bool, queue_size=10)


        rospy.Subscriber('/speech_transcription', String, self.speech_callback)
        rospy.Subscriber('/speech_done', Bool, self.tts_callback)
        rospy.Subscriber('/age_result', Int32, self.age_verification_callback)
        rospy.Subscriber('/customer_id_result', Int32, self.customer_id_callback)

        rospy.sleep(1.5)

        self.openai_api_key = "PLACE OPEN AI KEY HERE"
        self.model = "gpt-4"
        self.state = InitialState(parent=self, first_start=True)


    def make_openai_request(self, prompt, max_tokens=5):
        headers = {
            "Authorization": f"Bearer {self.openai_api_key}",
            "Content-Type": "application/json"
        }

        data = {
            "model": self.model,
            "messages": [{"role": "user", "content": prompt}],
            "max_tokens": max_tokens,
            "temperature": 0
        }

        try:
            response = requests.post("https://api.openai.com/v1/chat/completions",
                                     headers=headers, data=json.dumps(data))
            response.raise_for_status()
            reply = response.json()["choices"][0]["message"]["content"].strip().lower()
            rospy.loginfo(f"ChatGPT says: {reply}")
            return reply
        except Exception as e:
            rospy.logerr(f"Failed to contact OpenAI API: {e}")
            return None

    def tts_callback(self, result):
        rospy.loginfo("Speech done")
        self.state.tts_done()

    def say(self, text):
        rospy.loginfo(f"Robot says: {text}")
        say = String(text)
        self.tts_pub.publish(say)

    def trigger_stt(self):
        rospy.loginfo("Trigger STT")
        self.stt_pub.publish(Empty())

    def age_verification_callback(self, result):
        rospy.loginfo(f"Detected user age {result.data}") 
        self.age = result.data
        
    def customer_id_callback(self, result):
        rospy.loginfo(f"Detected customer {result.data}")
        self.user_id = f"user_{result.data}"

    def trigger_age_verification(self):
        rospy.loginfo("Age verification trigger")
        self.age_verification_pub.publish(True)

    def trigger_customer_id(self):
        rospy.loginfo("Customer id trigger")
        self.customer_id_pub.publish(True)

    def speech_callback(self, msg):
        rospy.loginfo(f"User said: {msg}")
        self.state = self.state.process_speech(msg)
        

    def simulate_movement(self):
        turn_cmd = Twist()
        turn_cmd.angular.z = 1.0 
        turn_duration = rospy.Duration(3.14) 

        start = rospy.Time.now()
        rate = rospy.Rate(10)
        while rospy.Time.now() - start < turn_duration:
            self.move_pub.publish(turn_cmd)
            rate.sleep()

        self.move_pub.publish(Twist())
        rospy.sleep(1.0)
        move_cmd = Twist()
        move_cmd.linear.x = 0.3
        forward_duration = rospy.Duration(2.0)  
        start = rospy.Time.now()
        while rospy.Time.now() - start < forward_duration:
            self.move_pub.publish(move_cmd)
            rate.sleep()
        self.move_pub.publish(Twist())


    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = ShopAssistant()
        node.run()
    except rospy.ROSInterruptException:
        pass
