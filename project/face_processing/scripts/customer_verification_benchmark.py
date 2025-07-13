#!/usr/bin/env python

import os
from collections import defaultdict
from face_processing import DeepFaceNode
import glob 
import sys
import termios
import tty

results = []
id_map = {}  # Map from person index to assigned customer ID

class CustomerVerificationBenchmark():
    def __init__(self, num_customers=2, attemps_per_customer=2):
        self.num_customers = num_customers
        self.attempts_per_customer = attemps_per_customer
        self.path = "/home/customer_verification_benchmark"
        self.deepface_node = DeepFaceNode(self.path)
        self.clear_customer_images(self.path)
    
    def clear_customer_images(self, directory):
        print(f"Cleaning up folder: {directory}")
        files = glob.glob(os.path.join(directory, "*.jpg"))
        for f in files:
            os.remove(f)
        print(f"Deleted {len(files)} files.")
    
    def wait_for_space(self):
        print("Press SPACE to start recording...")
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            while True:
                key = sys.stdin.read(1)
                if key == ' ':
                    break
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    def benchmark(self):
        print("=== Registration Phase ===")
        for person in range(self.num_customers):
            print(f"\n>> Person {person}, step up to the camera for registration.")
            self.wait_for_space()
            cid = self.deepface_node.customer_id()
            id_map[person] = cid
            print(f"Registered Person {person} with Customer ID {cid}")

        print("\n=== Testing Phase ===")
        for attempt in range(self.attempts_per_customer):
            for person in range(self.num_customers):
                print(f"\n>> Person {person}, please return for attempt {attempt+1}")
                self.wait_for_space()
                predicted_id = self.deepface_node.customer_id(create_new=False)

                expected_id = id_map[person]
                if predicted_id == expected_id:
                    outcome = "correct"
                elif predicted_id in id_map.values():
                    outcome = "confused"
                else:
                    outcome = "new"

                print(f"Predicted ID: {predicted_id}, Expected: {expected_id}, Outcome: {outcome}")
                results.append({
                    "expected": expected_id,
                    "predicted": predicted_id,
                    "outcome": outcome
                })

        return results

    def analyze(self, results):
        stats = defaultdict(int)
        for r in results:
            stats[r["outcome"]] += 1

        total = len(results)
        print("\n=== Benchmark Results ===")
        print(f"Total Attempts: {total}")
        print(f"Correct: {stats['correct']} ({stats['correct'] / total:.2%})")
        print(f"Confused: {stats['confused']} ({stats['confused'] / total:.2%})")
        print(f"New Registrations: {stats['new']} ({stats['new'] / total:.2%})")


if __name__ == "__main__":
    try:
        b = CustomerVerificationBenchmark()
        results = b.analyze(b.benchmark())
    except rospy.ROSInterruptException:
        pass
