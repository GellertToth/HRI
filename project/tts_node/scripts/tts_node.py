#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Bool
from TTS.api import TTS
import playsound
import os

class TTSNode:
    def __init__(self):
        rospy.init_node('tts_node')


        self.tts = TTS(model_name="tts_models/en/vctk/vits", )  
        rospy.Subscriber("/speak_text", String, self.handle_text)
        self.done_pub = rospy.Publisher("/speech_done", Bool, queue_size=1)

        rospy.loginfo("TTS Node started and ready to receive text.")
        rospy.spin()

    def handle_text(self, msg):
        text = msg.data.strip()
        if not text:
            rospy.logwarn("Received empty text, ignoring.")
            return

        rospy.loginfo(f"Received text to speak: {text}")

        wav_path = "/tmp/tts_output.wav"

        try:
            self.tts.tts_to_file(text=text, file_path=wav_path, speaker=self.tts.speakers[0])
            rospy.loginfo("Speech synthesis done, playing audio...")
            playsound.playsound(wav_path)

            rospy.loginfo("Audio playback finished.")
            self.done_pub.publish(True)
        except Exception as e:
            rospy.logerr(f"TTS or playback error: {e}")
            self.done_pub.publish(False)


if __name__ == '__main__':
    try:
        TTSNode()
    except rospy.ROSInterruptException:
        pass
