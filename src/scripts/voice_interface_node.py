import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import pyaudio
import io
import os
import threading
import time
import wave
from google.cloud import speech
from dotenv import load_dotenv

class VoiceInterfaceNode(Node):
    def __init__(self):
        super().__init__('voice_interface_node')
        
        # --- Configuration ---
        workspace_path = os.path.expanduser('~/kinova-gemini')
        load_dotenv(os.path.join(workspace_path, '.env'))
        self.api_key = os.getenv('gemini_api_key')

        # --- ROS2 Communication ---
        self.instruction_pub = self.create_publisher(String, '/user_instructions', 10)
        self.status_pub = self.create_publisher(String, '/voice_status', 10)
        self.trigger_sub = self.create_subscription(Bool, '/voice_trigger', self.trigger_callback, 10)

        # --- Audio Configuration ---
        self.CHUNK = 1024
        self.FORMAT = pyaudio.paInt16
        self.CHANNELS = 1
        self.RATE = 16000 # Standard for Google STT
        
        self.p = pyaudio.PyAudio()
        self.is_recording = False
        self.recording_thread = None
        self.audio_frames = []

        # --- STT Client ---
        try:
            # Try to use API key for authentication
            self.client = speech.SpeechClient(
                client_options={"api_key": self.api_key}
            )
            self.get_logger().info("STT Client initialized with API key.")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize STT Client: {e}")
            self.client = None

        self.get_logger().info("Voice Interface Node initialized.")
        self.status_pub.publish(String(data="IDLE"))

    def trigger_callback(self, msg):
        if msg.data:
            if not self.is_recording:
                self.start_recording()
        else:
            if self.is_recording:
                self.stop_recording()

    def start_recording(self):
        self.get_logger().info("Starting recording...")
        self.is_recording = True
        self.audio_frames = []
        self.status_pub.publish(String(data="LISTENING"))
        
        self.recording_thread = threading.Thread(target=self.record_loop)
        self.recording_thread.start()

    def record_loop(self):
        try:
            stream = self.p.open(format=self.FORMAT,
                                channels=self.CHANNELS,
                                rate=self.RATE,
                                input=True,
                                frames_per_buffer=self.CHUNK)
            
            while self.is_recording:
                data = stream.read(self.CHUNK, exception_on_overflow=False)
                self.audio_frames.append(data)
            
            stream.stop_stream()
            stream.close()
        except Exception as e:
            self.get_logger().error(f"Error in recording loop: {e}")
            self.is_recording = False

    def stop_recording(self):
        self.get_logger().info("Stopping recording and processing...")
        self.is_recording = False
        if self.recording_thread:
            self.recording_thread.join()
        
        self.status_pub.publish(String(data="PROCESSING"))
        
        # Process audio in a separate thread to not block the callback
        threading.Thread(target=self.process_audio).start()

    def process_audio(self):
        if not self.audio_frames:
            self.get_logger().warn("No audio captured.")
            self.status_pub.publish(String(data="IDLE"))
            return

        # Convert frames to bytes
        audio_content = b''.join(self.audio_frames)
        
        # Prepare the request
        audio = speech.RecognitionAudio(content=audio_content)
        config = speech.RecognitionConfig(
            encoding=speech.RecognitionConfig.AudioEncoding.LINEAR16,
            sample_rate_hertz=self.RATE,
            language_code="en-US",
        )

        try:
            if not self.client:
                 raise Exception("STT Client not initialized.")

            response = self.client.recognize(config=config, audio=audio)

            if response.results:
                # Get the best transcript
                transcript = response.results[0].alternatives[0].transcript
                confidence = response.results[0].alternatives[0].confidence
                
                self.get_logger().info(f"Transcribed: '{transcript}' (confidence: {confidence:.2f})")
                
                # Publish to /user_instructions
                msg = String()
                msg.data = transcript
                self.instruction_pub.publish(msg)
            else:
                self.get_logger().warn("STT returned no results.")

        except Exception as e:
            self.get_logger().error(f"STT error: {e}")

        self.status_pub.publish(String(data="IDLE"))

def main(args=None):
    rclpy.init(args=args)
    node = VoiceInterfaceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
