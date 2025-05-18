#!/usr/bin/env python3
import rospy
from std_msgs.msg import String, Empty
import whisper
import tempfile
import sounddevice as sd
import numpy as np
import threading
import torch
import os
import wave

class WhisperSpeechToTextNode:
    def __init__(self):
        rospy.init_node('whisper_speech_to_text_node')
        self.listen_duration = rospy.get_param('~listen_duration', 5.0)
        self.samplerate = 16000  # Whisper expects 16 kHz
        self.channels = 1

        self.transcription_pub = rospy.Publisher('/speech_transcription', String, queue_size=10)
        rospy.Subscriber('/start_listening', Empty, self.listen_callback)

        device = "cuda" if torch.cuda.is_available() else "cpu"
        rospy.loginfo(f"Loading Whisper model on {device}...")
        self.model = whisper.load_model("medium", device=device)
        rospy.loginfo("Whisper model loaded.")

    def listen_callback(self, msg):
        rospy.loginfo(f"Received start signal. Listening for {self.listen_duration} seconds.")
        threading.Thread(target=self.capture_and_transcribe).start()

    def capture_and_transcribe(self):
        try:
            audio_data = self.record_audio()
            with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as f:
                self.save_audio_to_wav(audio_data, f.name)
                audio_path = f.name

            rospy.loginfo("Transcribing...")
            result = self.model.transcribe(audio_path, fp16=torch.cuda.is_available())
            transcription = result["text"]
            rospy.loginfo(f"Transcription: {transcription}")
            self.transcription_pub.publish(transcription)

            os.remove(audio_path)

        except Exception as e:
            rospy.logerr(f"Error during transcription: {e}")
            self.transcription_pub.publish("")

    def record_audio(self):
        rospy.loginfo("Recording audio...")
        audio = sd.rec(int(self.listen_duration * self.samplerate),
                       samplerate=self.samplerate, channels=self.channels, dtype='int16')
        sd.wait()
        rospy.loginfo("Recording complete.")
        return audio

    def save_audio_to_wav(self, audio_data, filename):
        with wave.open(filename, 'wb') as wf:
            wf.setnchannels(self.channels)
            wf.setsampwidth(2)  # 'int16' = 2 bytes
            wf.setframerate(self.samplerate)
            wf.writeframes(audio_data.tobytes())

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = WhisperSpeechToTextNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
