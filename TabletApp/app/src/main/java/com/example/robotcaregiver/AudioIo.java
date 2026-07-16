package com.example.robotcaregiver.live;

import android.annotation.SuppressLint;
import android.media.AudioAttributes;
import android.media.AudioFormat;
import android.media.AudioManager;
import android.media.AudioRecord;
import android.media.AudioTrack;
import android.media.MediaRecorder;
import android.util.Log;

import androidx.annotation.NonNull;

/**
 * Microphone capture and speaker playback for a Live session.
 * Barge-in: when LiveSession reports onInterrupted(), call flushPlayback()
 *
 */
public class AudioIo {

    private static final String TAG = "AudioIo";
    private static final int IN_RATE = 16000;
    private static final int OUT_RATE = 24000;

    public interface MicListener {
        void onPcmChunk(@NonNull byte[] data, int len);
    }

    private AudioRecord record;
    private AudioTrack track;
    private volatile boolean capturing;
    private Thread captureThread;

    // ---- Capture (mic) ----

    @SuppressLint("MissingPermission") // checked in MainActivity
    public void startCapture(@NonNull MicListener listener) {
        int minBuf = AudioRecord.getMinBufferSize(
                IN_RATE, AudioFormat.CHANNEL_IN_MONO, AudioFormat.ENCODING_PCM_16BIT);

        int chunkBytes = 2048;
        record = new AudioRecord(
                MediaRecorder.AudioSource.VOICE_COMMUNICATION, // echo-cancelled path
                IN_RATE, AudioFormat.CHANNEL_IN_MONO,
                AudioFormat.ENCODING_PCM_16BIT, Math.max(minBuf, chunkBytes * 4));

        record.startRecording();
        capturing = true;
        captureThread = new Thread(() -> {
            byte[] buf = new byte[chunkBytes];
            while (capturing) {
                int n = record.read(buf, 0, buf.length);
                if (n > 0) listener.onPcmChunk(buf, n);
            }
        }, "mic-capture");
        captureThread.start();
    }

    public void stopCapture() {
        capturing = false;
        try { if (captureThread != null) captureThread.join(300); } catch (InterruptedException ignored) {}
        if (record != null) {
            try { record.stop(); } catch (Exception ignored) {}
            record.release();
            record = null;
        }
    }

    public void initPlayback() {
        int minBuf = AudioTrack.getMinBufferSize(
                OUT_RATE, AudioFormat.CHANNEL_OUT_MONO, AudioFormat.ENCODING_PCM_16BIT);
        track = new AudioTrack.Builder()
                .setAudioAttributes(new AudioAttributes.Builder()
                        .setUsage(AudioAttributes.USAGE_VOICE_COMMUNICATION)
                        .setContentType(AudioAttributes.CONTENT_TYPE_SPEECH)
                        .build())
                .setAudioFormat(new AudioFormat.Builder()
                        .setEncoding(AudioFormat.ENCODING_PCM_16BIT)
                        .setSampleRate(OUT_RATE)
                        .setChannelMask(AudioFormat.CHANNEL_OUT_MONO)
                        .build())
                .setBufferSizeInBytes(Math.max(minBuf, OUT_RATE)) // ~0.5s cushion
                .setTransferMode(AudioTrack.MODE_STREAM)
                .build();
        track.play();
    }

    public void playChunk(@NonNull byte[] pcm24k) {
        if (track != null) track.write(pcm24k, 0, pcm24k.length);
    }

    public void flushPlayback() {
        if (track != null) {
            try {
                track.pause();
                track.flush();
                track.play();
            } catch (Exception e) {
                Log.e(TAG, "flushPlayback", e);
            }
        }
    }

    public void releasePlayback() {
        if (track != null) {
            try { track.stop(); } catch (Exception ignored) {}
            track.release();
            track = null;
        }
    }
}
