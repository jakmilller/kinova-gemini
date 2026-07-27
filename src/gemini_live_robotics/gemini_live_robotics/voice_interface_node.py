#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import pyaudio
import io
import os
import threading
import wave
import yaml
import numpy as np
from google import genai
from google.genai import types
from dotenv import load_dotenv

# faster-whisper is optional — the node still runs (via the Gemini fallback) without it.
try:
    from faster_whisper import WhisperModel
    WHISPER_AVAILABLE = True
except ImportError:
    WHISPER_AVAILABLE = False

# Push-to-talk voice input. Records mic audio while /voice_trigger is True (button held),
# and on release transcribes the clip, publishing the text to /user_instructions so it
# enters the brain exactly like a typed command. Transcription is local faster-whisper by
# default (fast, no network); it falls back to a Gemini API call if whisper is unavailable.
#
#   /voice_trigger (Bool)  in:  True = start recording, False = stop + transcribe
#   /user_instructions (String) out: the transcript
#   /voice_status (String) out: IDLE / LISTENING / PROCESSING (drives the web UI indicator)


class VoiceInterfaceNode(Node):
    def __init__(self):
        super().__init__('voice_interface_node')

        # --- Configuration (same key + model the brain node uses) ---
        workspace_path = os.path.expanduser('~/kinova-gemini')
        load_dotenv(os.path.join(workspace_path, '.env'))
        self.api_key = os.getenv('gemini_api_key')

        with open(os.path.join(workspace_path, 'config.yaml'), 'r') as f:
            config = yaml.safe_load(f)
        # Gemini model used only as the cloud fallback when local whisper is unavailable.
        self.model_id = config['model'].get('transcribe') or config['model']['flash']
        device_hint = ((config.get('audio') or {}).get('input_device') or "").strip()
        self.tcfg = config.get('transcription') or {}

        # --- ROS2 Communication ---
        self.instruction_pub = self.create_publisher(String, '/user_instructions', 10)
        self.status_pub = self.create_publisher(String, '/voice_status', 10)
        self.trigger_sub = self.create_subscription(Bool, '/voice_trigger', self.trigger_callback, 10)

        # --- Audio Configuration ---
        self.CHUNK = 1024
        self.FORMAT = pyaudio.paInt16
        self.CHANNELS = 1
        self.RATE = 16000  # 16kHz mono is plenty for speech

        self.p = pyaudio.PyAudio()
        self.input_device_index = self._resolve_input_device(device_hint)
        self.is_recording = False
        self.recording_thread = None
        self.audio_frames = []

        # --- Gemini client (always available as the transcription fallback) ---
        try:
            self.client = genai.Client(api_key=self.api_key)
        except Exception as e:
            self.get_logger().error(f"Failed to initialize Gemini client: {e}")
            self.client = None

        # --- Local whisper model (preferred: no network round trip) ---
        self.whisper_model = self._load_whisper()

        backend = "faster-whisper (local)" if self.whisper_model is not None \
            else f"Gemini API ({self.model_id})"
        self.get_logger().info(f"Voice Interface Node ready (Push-to-Talk). Transcriber: {backend}.")
        self.status_pub.publish(String(data="IDLE"))

    def _load_whisper(self):
        """Load faster-whisper once at startup and warm it up. Returns the model, or None to
        signal 'use the Gemini fallback'. Degrades gracefully: cuda -> cpu -> None, so a
        missing dependency or tight VRAM can never hard-fail a voice command."""
        if self.tcfg.get('backend', 'whisper') != 'whisper' or not WHISPER_AVAILABLE:
            if not WHISPER_AVAILABLE and self.tcfg.get('backend', 'whisper') == 'whisper':
                self.get_logger().warn("faster-whisper not installed — using Gemini fallback.")
            return None

        model_name = self.tcfg.get('whisper_model', 'base.en')
        device = self.tcfg.get('whisper_device', 'cuda')
        compute_type = self.tcfg.get('whisper_compute_type', 'int8_float16')

        # First choice is the configured device; on any failure retry on CPU before giving up.
        for dev, ctype in ((device, compute_type), ('cpu', 'int8')):
            try:
                self.get_logger().info(f"Loading whisper '{model_name}' on {dev} ({ctype})...")
                model = WhisperModel(model_name, device=dev, compute_type=ctype)
                # Warm-up: the first transcription otherwise pays lazy CUDA-kernel/model init.
                model.transcribe(np.zeros(self.RATE // 2, dtype=np.float32), language="en", beam_size=1)
                self.get_logger().info(f"Whisper ready on {dev}.")
                return model
            except Exception as e:
                self.get_logger().warn(f"Whisper load on {dev} failed: {e}")
        self.get_logger().warn("Whisper unavailable — using Gemini fallback.")
        return None

    def _resolve_input_device(self, name_hint):
        """None = PyAudio default (follows the system default source). Only override if a
        named device is requested and found."""
        if not name_hint:
            return None
        for i in range(self.p.get_device_count()):
            info = self.p.get_device_info_by_index(i)
            if info['maxInputChannels'] > 0 and name_hint.lower() in info['name'].lower():
                self.get_logger().info(f"Using mic [{i}]: {info['name']}")
                return i
        self.get_logger().warn(f"No mic matching '{name_hint}'; using system default.")
        return None

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
                                 frames_per_buffer=self.CHUNK,
                                 input_device_index=self.input_device_index)

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

        # Process audio off the callback thread so we don't block incoming triggers.
        threading.Thread(target=self.process_audio).start()

    def _frames_to_wav(self):
        """Wrap the captured raw PCM in an in-memory WAV container so the transcription
        request is self-describing (rate/format live in the WAV header)."""
        buffer = io.BytesIO()
        with wave.open(buffer, 'wb') as wf:
            wf.setnchannels(self.CHANNELS)
            wf.setsampwidth(self.p.get_sample_size(self.FORMAT))
            wf.setframerate(self.RATE)
            wf.writeframes(b''.join(self.audio_frames))
        return buffer.getvalue()

    def _transcribe_whisper(self):
        """Local transcription. faster-whisper takes a float32 mono array directly, so no WAV
        encoding is needed. beam_size=1 keeps latency minimal."""
        audio = np.frombuffer(b''.join(self.audio_frames), np.int16).astype(np.float32) / 32768.0
        segments, _ = self.whisper_model.transcribe(audio, language="en", beam_size=1)
        return "".join(seg.text for seg in segments).strip()

    def _transcribe_gemini(self):
        """Cloud fallback. Wraps the PCM in a WAV container (self-describing) and asks Gemini."""
        if not self.client:
            raise Exception("Gemini client not initialized.")
        wav_bytes = self._frames_to_wav()
        response = self.client.models.generate_content(
            model=self.model_id,
            contents=[
                "Transcribe this audio to text. Respond with ONLY the transcription, "
                "no extra words, quotes, or punctuation commentary.",
                types.Part.from_bytes(data=wav_bytes, mime_type="audio/wav"),
            ],
        )
        return (response.text or "").strip()

    def process_audio(self):
        if not self.audio_frames:
            self.get_logger().warn("No audio captured.")
            self.status_pub.publish(String(data="IDLE"))
            return

        try:
            if self.whisper_model is not None:
                transcript = self._transcribe_whisper()
            else:
                transcript = self._transcribe_gemini()

            if transcript:
                self.get_logger().info(f"Transcribed: '{transcript}'")
                self.instruction_pub.publish(String(data=transcript))
            else:
                self.get_logger().warn("Transcription returned no text.")

        except Exception as e:
            self.get_logger().error(f"Transcription error: {e}")

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
