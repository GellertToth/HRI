#!/usr/bin/env python

import rospy
from jiwer import wer
import time
import sys
import termios
import tty
import os

from STT_node import WhisperSpeechToTextNode

class STTBenchmarkNode:
    def __init__(self):
        self.STT = WhisperSpeechToTextNode()

        self.sentences = [
            "Where can I find the organic bananas?",
            "Do you have any gluten free pasta?",
            "I'm looking for almond milk, can you help?",
            "What aisle is the rice in?",
            "Are there any discounts on chicken today?",
            "Can you tell me the ingredients in this soup?",
            "Is this cereal suitable for vegans?",
            "Do you sell lactose free yogurt?",
            "I need help finding the baby formula.",
            "How much does this bottle of olive oil cost?",
            "Are the strawberries on sale this week?",
            "Can you show me where the cleaning supplies are?",
        ]

        self.run_benchmark()

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

    def run_benchmark(self):
        total_wer = 0.0
        total_tests = 0

        for idx, sentence in enumerate(self.sentences):
            rospy.loginfo("\n====================================")
            rospy.loginfo(f"Sentence {idx+1}: {sentence}")
            rospy.loginfo("Please read the sentence aloud...")

            self.wait_for_space()

            try:
                transcription = self.STT.capture_and_transcribe()
                error = wer(sentence.lower(), transcription.lower())
                rospy.loginfo(f"WER: {error:.2f}")
                total_wer += error
                total_tests += 1
            except Exception as e:
                rospy.logerr(f"Error capturing or transcribing: {e}")

            time.sleep(1.0)

        if total_tests > 0:
            avg_wer = total_wer / total_tests
            rospy.loginfo("====================================")
            rospy.loginfo(f"Benchmark completed. Average WER: {avg_wer:.2f}")
        else:
            rospy.logwarn("No successful transcriptions. Benchmark failed.")

        rospy.signal_shutdown("Benchmark complete")


if __name__ == "__main__":
    try:
        STTBenchmarkNode()
    except rospy.ROSInterruptException:
        pass
