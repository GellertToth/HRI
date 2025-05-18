#!/usr/bin/env python3


import rospy
from std_msgs.msg import String
import sounddevice as sd
import numpy as np
import whisper
import time
import queue
import torch

# Constants
SAMPLE_RATE = 16000  # 16kHz audio
BLOCK_SIZE = 512     # Process 512 samples per block (32ms audio chunk)
BUFFER_SIZE = 2 * SAMPLE_RATE  # Buffer size for 3 seconds of audio
TRANSCRIPTION_INTERVAL = 1  # Process every 1 second

# Queue to hold audio chunks
audio_queue = queue.Queue()

# Check if GPU is available
device = "cuda" if torch.cuda.is_available() else "cpu"
print(device)
print(f"Using device: {device}")

# Load Whisper model (use "base" or other models like "small", "medium", "large")
model = whisper.load_model("base").to(device)  # Move model to GPU if available

# Audio buffer to store the last 3 seconds of audio
audio_buffer = np.zeros(BUFFER_SIZE, dtype=np.float32)

# ROS Publisher setup
rospy.init_node('realtime_stt_node')
pub = rospy.Publisher('/speech_text', String, queue_size=10)

# Audio callback to fill queue
def callback(indata, frames, time_, status):
    if status:
        print(status)
    audio_queue.put(indata.copy())

# Stream setup
stream = sd.InputStream(samplerate=SAMPLE_RATE, channels=1, dtype='float32', callback=callback, blocksize=BLOCK_SIZE)

# Function to process audio buffer and transcribe it
def process_audio():
    while not rospy.is_shutdown():
        # Fetch the latest audio chunk
        audio_chunk = audio_queue.get()

        # Flatten the audio chunk to a 1D array if it's a 2D array
        audio_chunk = audio_chunk.flatten()

        # Shift the old audio buffer and append the new chunk
        audio_buffer[:-audio_chunk.shape[0]] = audio_buffer[audio_chunk.shape[0]:]
        audio_buffer[-audio_chunk.shape[0]:] = audio_chunk

        # Every second, transcribe the last 3 seconds of audio
        if time.time() % TRANSCRIPTION_INTERVAL < 0.1:
            # Prepare the audio data for transcription
            audio_data = np.clip(audio_buffer, -1.0, 1.0)  # Ensure it's within [-1, 1]
            audio_data = torch.from_numpy(audio_data).to(device)  # Convert to torch tensor and move to GPU if available

            # Use Whisper model to transcribe the audio
            result = model.transcribe(audio_data.numpy())  # Convert back to numpy for transcribing
            print("Transcription:", result['text'])

            if(len(result["text"]) > 0):
                pub.publish(result['text'])

# Start the audio stream and transcription process
if __name__ == '__main__':
    try:
        with stream:
            process_audio()
    except rospy.ROSInterruptException:
        print("ROS Node interrupted.")
