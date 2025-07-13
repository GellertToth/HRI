#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Int32
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
from deepface import DeepFace

class DeepFaceNode:
    def __init__(self, know_customers_path= "/home/tiago_public_ws/src/project/known_customers"):
        rospy.init_node('deepface_age_estimator')

        self.bridge = CvBridge()
        self.image = None
        self.known_dir = know_customers_path

        rospy.Subscriber("/usb_cam/image_raw", Image, self.image_callback)
        rospy.Subscriber("/check_age", Bool, self.check_age_callback)
        rospy.Subscriber('/customer_id', Bool, self.get_customer_id_callback)
        self.age_pub = rospy.Publisher("/age_result", Int32, queue_size=1)
        self.customer_id_pub = rospy.Publisher("/customer_id_result", Int32, queue_size=1)


        rospy.loginfo("DeepFace Age Estimator Node started.")

    def run(self):
        rospy.spin()

    def image_callback(self, msg):
        try:
            self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            rospy.logerr(f"Failed to convert image: {e}")

    def get_next_customer_id(self, dir_path):
        existing = [f for f in os.listdir(dir_path) if f.startswith("customer_")]
        ids = [int(f.split("_")[1].split(".")[0]) for f in existing]
        return max(ids + [-1]) + 1

    def customer_id(self, create_new=True):
        if self.image is None:
            rospy.logwarn("No image available for customer ID check.")
            return

        os.makedirs(self.known_dir, exist_ok=True)

        temp_path = "/tmp/current_customer.jpg"
        cv2.imwrite(temp_path, self.image)

        best_score = float("inf")
        best_id = None
        threshold = 0.3  # adjust for sensitivity

        # Compare to each saved customer
        for file in os.listdir(self.known_dir):
            if not file.endswith(".jpg"):
                continue
            path = os.path.join(self.known_dir, file)
            try:
                result = DeepFace.verify(temp_path, path, enforce_detection=False)
                if result["verified"] and result["distance"] < best_score:
                    best_score = result["distance"]
                    best_id = file
            except Exception as e:
                rospy.logwarn(f"Error comparing to {file}: {e}")

        if best_score < threshold and best_id is not None:
            rospy.loginfo(f"Recognized customer: {best_id} with distance {best_score:.3f}")
            customer_num = int(best_id.split("_")[1].split(".")[0])
        else:
            customer_num = self.get_next_customer_id(self.known_dir)
            if create_new:
                new_path = os.path.join(self.known_dir, f"customer_{customer_num}.jpg")
                cv2.imwrite(new_path, self.image)
                rospy.loginfo(f"Registered new customer: customer_{customer_num}")
        return customer_num

    def get_customer_id_callback(self, msg):
        customer_num = self.customer_id()
        self.customer_id_pub.publish(customer_num)


    def check_age_callback(self, msg):
        if not msg.data or self.image is None:
            rospy.logwarn("No trigger or no image available.")
            self.age_pub.publish(-1)
            return

        # Save the frame to a temporary file (DeepFace prefers paths, not raw arrays)
        temp_path = "/tmp/temp_frame.jpg"
        cv2.imwrite(temp_path, self.image)

        try:
            result = DeepFace.analyze(img_path=temp_path, actions=['age'], enforce_detection=False)
            age = int(result[0]['age'])
            age = 10
            rospy.loginfo(f"Predicted age: {age}")
            self.age_pub.publish(age)
        except Exception as e:
            rospy.logerr(f"DeepFace analysis failed: {e}")

if __name__ == '__main__':
    try:
        node = DeepFaceNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
