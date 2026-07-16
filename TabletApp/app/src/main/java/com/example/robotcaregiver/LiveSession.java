package com.example.robotcaregiver.live;

import android.util.Base64;
import android.util.Log;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import org.json.JSONArray;
import org.json.JSONObject;

import okhttp3.OkHttpClient;
import okhttp3.Request;
import okhttp3.Response;
import okhttp3.WebSocket;
import okhttp3.WebSocketListener;
import okio.ByteString;

/**
 * Manages Gemini Live WebSocket session.
 *
 *   - Connect to URL from LiveAuthProvider.
 *   - Setup message with model, generation_config, system_instruction, and tools.
 *   - Stream user input via realtimeInput: audio, camera JPEG.
 */
public class LiveSession {

    private static final String TAG = "LiveSession";

    // Live-capable model. NOTE: different from the standard API model name, and
    // Live model names rotate quickly. If connect closes with code 1008
    // ("model not found / not supported for bidiGenerateContent"), update this.
    private static final String MODEL = "models/gemini-2.5-flash-native-audio-preview-12-2025";

    public interface Listener {
        void onReady();
        void onAudioOutput(@NonNull byte[] pcm24k);   // play this
        void onTranscript(@NonNull String text, boolean fromUser);
        void onToolCall(@NonNull String name, @NonNull String id, @NonNull JSONObject args);
        void onInterrupted();                          // barge-in: purge playback
        void onClosed(@NonNull String reason);
        void onError(@NonNull String message);
    }

    private final OkHttpClient client;
    private final Listener listener;
    private WebSocket ws;

    public LiveSession(@NonNull Listener listener) {
        this.listener = listener;
        this.client = new OkHttpClient.Builder()
                .pingInterval(60, java.util.concurrent.TimeUnit.SECONDS) // keep ws alive
                .build();
    }

    public void connect(@NonNull String wssUrl) {
        Request req = new Request.Builder().url(wssUrl).build();
        ws = client.newWebSocket(req, new WebSocketListener() {
            @Override
            public void onOpen(@NonNull WebSocket webSocket, @NonNull Response response) {
                sendSetup();
            }

            @Override
            public void onMessage(@NonNull WebSocket webSocket, @NonNull String text) {
                handleServerMessage(text);
            }

            @Override
            public void onMessage(@NonNull WebSocket webSocket, @NonNull ByteString bytes) {
                handleServerMessage(bytes.utf8()); // server may send binary JSON
            }

            @Override
            public void onClosing(@NonNull WebSocket webSocket, int code, @NonNull String reason) {
                listener.onClosed("ws closing " + code + ": " + reason);
            }

            @Override
            public void onFailure(@NonNull WebSocket webSocket,
                                  @NonNull Throwable t, @Nullable Response response) {
                listener.onError("ws failure: " + t.getMessage());
            }
        });
    }

    private void sendSetup() {
        try {
            JSONObject setup = new JSONObject()
                    .put("model", MODEL)
                    .put("generation_config", new JSONObject()
                            .put("response_modalities", new JSONArray().put("AUDIO")));


            JSONObject msg = new JSONObject().put("setup", setup);
            ws.send(msg.toString());
            listener.onReady();
        } catch (Exception e) {
            listener.onError("setup failed: " + e.getMessage());
        }
    }

    public void sendAudio(@NonNull byte[] pcm16k, int len) {
        try {
            String b64 = Base64.encodeToString(pcm16k, 0, len, Base64.NO_WRAP);
            JSONObject blob = new JSONObject()
                    .put("data", b64)
                    .put("mime_type", "audio/pcm;rate=16000");
            JSONObject msg = new JSONObject().put("realtimeInput",
                    new JSONObject().put("audio", blob));
            if (ws != null) ws.send(msg.toString());
        } catch (Exception e) {
            Log.e(TAG, "sendAudio", e);
        }
    }

    public void sendVideoFrame(@NonNull byte[] jpeg) {
        try {
            String b64 = Base64.encodeToString(jpeg, Base64.NO_WRAP);
            JSONObject blob = new JSONObject()
                    .put("data", b64)
                    .put("mime_type", "image/jpeg");
            JSONObject msg = new JSONObject().put("realtimeInput",
                    new JSONObject().put("video", blob));
            if (ws != null) ws.send(msg.toString());
        } catch (Exception e) {
            Log.e(TAG, "sendVideoFrame", e);
        }
    }

    public void sendToolResponse(@NonNull String name, @NonNull String id,
                                 @NonNull JSONObject response) {
        try {
            JSONObject fr = new JSONObject()
                    .put("name", name).put("id", id).put("response", response);
            JSONObject msg = new JSONObject().put("toolResponse",
                    new JSONObject().put("functionResponses", new JSONArray().put(fr)));
            if (ws != null) ws.send(msg.toString());
        } catch (Exception e) {
            Log.e(TAG, "sendToolResponse", e);
        }
    }

    private void handleServerMessage(@NonNull String json) {
        try {
            android.util.Log.d("RAWMSG", json);
            JSONObject root = new JSONObject(json);

            JSONObject sc = root.optJSONObject("serverContent");
            if (sc != null) {
                if (sc.optBoolean("interrupted", false)) {
                    listener.onInterrupted(); // barge-in: caller purges audio now
                }
                // audio output & text parts
                JSONObject modelTurn = sc.optJSONObject("modelTurn");
                if (modelTurn != null) {
                    JSONArray parts = modelTurn.optJSONArray("parts");
                    if (parts != null) {
                        for (int i = 0; i < parts.length(); i++) {
                            JSONObject p = parts.getJSONObject(i);
                            JSONObject inline = p.optJSONObject("inlineData");
                            if (inline != null) {
                                String mime = inline.optString("mimeType", "");
                                if (mime.startsWith("audio")) {
                                    byte[] pcm = Base64.decode(
                                            inline.optString("data", ""), Base64.DEFAULT);
                                    listener.onAudioOutput(pcm);
                                }
                            }
                            String t = p.optString("text", "");
//                            if (!t.isEmpty()) {
//                                listener.onTranscript(t, false);
//                            }
                        }
                    }
                }
                JSONObject outTr = sc.optJSONObject("outputTranscription");
                if (outTr != null) listener.onTranscript(outTr.optString("text", ""), false);
                JSONObject inTr = sc.optJSONObject("inputTranscription");
                if (inTr != null) listener.onTranscript(inTr.optString("text", ""), true);
            }

            JSONObject toolCall = root.optJSONObject("toolCall");
            if (toolCall != null) {
                JSONArray fcs = toolCall.optJSONArray("functionCalls");
                if (fcs != null) {
                    for (int i = 0; i < fcs.length(); i++) {
                        JSONObject fc = fcs.getJSONObject(i);
                        listener.onToolCall(
                                fc.optString("name", ""),
                                fc.optString("id", ""),
                                fc.optJSONObject("args") != null
                                        ? fc.getJSONObject("args") : new JSONObject());
                    }
                }
            }
        } catch (Exception e) {
            listener.onError("parse: " + e.getMessage());
        }
    }

    public void close() {
        if (ws != null) {
            ws.close(1000, "client closing");
            ws = null;
        }
    }
}
