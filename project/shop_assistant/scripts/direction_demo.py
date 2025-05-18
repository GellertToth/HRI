#!/usr/bin/env python3
import rospy
from std_msgs.msg import String, Empty
from geometry_msgs.msg import Twist
from sound_play.msg import SoundRequest
import requests
import json
import time
import random

class ShopAssistant:
    def __init__(self):
        rospy.init_node('shop_assistant_node')

        self.stt_pub = rospy.Publisher('/start_listening', Empty, queue_size=10)
        self.say_pub = rospy.Publisher('/robotsound', SoundRequest, queue_size=10)
        self.move_pub = rospy.Publisher('/mobile_base_controller/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/speech_transcription', String, self.speech_callback)


        self.awaiting_transcription = False
        self.state = "init"
        self.last_input = ""

        # Replace with your OpenAI API key
        self.openai_api_key = "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"
        self.model = "gpt-4"

        rospy.sleep(1.0)
        self.start_conversation()

    def start_conversation(self):
        self.say("How can I help you?")
        rospy.sleep(2)
        self.trigger_stt()
        self.state = "waiting_for_request"

    def say(self, text):
        rospy.loginfo(f"Robot says: {text}")
        say = SoundRequest()
        say.sound = SoundRequest.SAY
        say.command = SoundRequest.PLAY_ONCE
        say.arg = text
        say.volume = 1.0
        self.say_pub.publish(say)

    def trigger_stt(self):
        rospy.sleep(0.5)
        self.awaiting_transcription = True
        self.stt_pub.publish(Empty())

    def speech_callback(self, msg):
        if not self.awaiting_transcription:
            return

        text = msg.data.strip()
        if not text:
            rospy.logwarn("Received empty transcription.")
            return

        rospy.loginfo(f"User said: {text}")
        self.last_input = text
        self.awaiting_transcription = False

        if self.state == "waiting_for_request":
            self.say("You can find that in Aisle 4, to your left.")
            rospy.sleep(4)
            self.say("Did that help?")
            rospy.sleep(1)
            self.trigger_stt()
            self.state = "waiting_for_feedback"

        elif self.state == "waiting_for_feedback":
            rospy.loginfo("Analyzing sentiment...")
            if self.check_user_satisfied(text):
                self.say("Glad I could help! Have a nice day.")
                rospy.signal_shutdown("Conversation complete.")
            else:
                self.say("Follow me.")
                self.simulate_movement()
                rospy.signal_shutdown("Leading customer.")

    def check_user_satisfied(self, user_input):
        prompt = f"""
            A customer just said: "{user_input}"
            Are they satisfied with the directions given by the assistant? Answer with "yes" or "no" only.
        """

        headers = {
            "Authorization": f"Bearer {self.openai_api_key}",
            "Content-Type": "application/json"
        }

        data = {
            "model": self.model,
            "messages": [{"role": "user", "content": prompt}],
            "max_tokens": 5,
            "temperature": 0
        }

        try:
            response = requests.post("https://api.openai.com/v1/chat/completions",
                                     headers=headers, data=json.dumps(data))
            response.raise_for_status()
            reply = response.json()["choices"][0]["message"]["content"].strip().lower()
            rospy.loginfo(f"ChatGPT says: {reply}")
            return "yes" in reply
        except Exception as e:
            rospy.logerr(f"Failed to contact OpenAI API: {e}")
            return False

    def simulate_movement(self):
        # Replace with real navigation calls in actual deployment
        move_cmd = Twist()
        move_cmd.linear.x = 0.3
        move_cmd.angular.z = 0.0
        start = rospy.Time.now()
        rate = rospy.Rate(10)
        while rospy.Time.now() - start < rospy.Duration(2.0):
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
