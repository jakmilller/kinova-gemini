//package com.example.robotcaregiver;
//
//import static com.example.robotcaregiver.BuildConfig.GEMINI_API_KEY;
//
//import android.Manifest;
//import android.content.Intent;
//import android.content.pm.PackageManager;
//import android.graphics.Bitmap;
//import android.graphics.BitmapFactory;
//import android.os.Build;
//import android.os.Bundle;
//import android.widget.Button;
//import android.widget.ImageView;
//import android.widget.TextView;
//import android.widget.Toast;
//
//import androidx.annotation.NonNull;
//import androidx.annotation.Nullable;
//import androidx.appcompat.app.AppCompatActivity;
//import androidx.core.app.ActivityCompat;
//
//import com.example.robotcaregiver.live.AudioIo;
//import com.example.robotcaregiver.live.LiveAuthProvider;
//import com.example.robotcaregiver.live.LiveSession;
//import com.example.robotcaregiver.live.*;
//
//import org.json.JSONObject;
//
//public class MainActivityBrainOnTablet extends AppCompatActivity
//        implements LiveSession.Listener, KitLink.Listener {
//
//    private TextView statusView;
//    private ImageView frameView;
//    private Button startStopButton;
//    private Button settingsButton;
//
//    private LiveAuthProvider auth;
//    private LiveSession live;
//    private final AudioIo audio = new AudioIo();
//    private KitLink kit;
//    private final StringBuilder spokenMessage = new StringBuilder();
//
//    private boolean running;
//
//    @Override
//    protected void onCreate(@Nullable Bundle savedInstanceState) {
//        super.onCreate(savedInstanceState);
//        setContentView(R.layout.activity_main);
//
//        statusView = findViewById(R.id.statusView);
//        frameView = findViewById(R.id.frameView);
//        startStopButton = findViewById(R.id.startStopButton);
//        settingsButton = findViewById(R.id.settingsButton);
//
//        AppSettings settings = new AppSettings(this);
//
//        if(!settings.isConfigured()){
//            startActivity(new Intent(this, SettingsActivity.class));
//        }
//
//        auth = new EphemeralTokenAuthProvider("http://" + settings.getBackendUrl() + ":8000/api/token", "X-App-Token", BuildConfig.APP_SHARED);
//
//        kit = new BluetoothKitLink(settings.getKitMac()); // kit's MAC
//        kit.setListener(this);
//
//        requestRuntimePermissions();
//        kit.connect();
//
//        startStopButton.setOnClickListener(v -> { if (running) stopSession(); else startSession(); });
//        settingsButton.setOnClickListener(v -> startActivity(new Intent(this, SettingsActivity.class)));
//    }
//
//    private void startSession() {
//        setStatus("Connecting to Gemini Live...");
//        auth.getConnectionUrl(new LiveAuthProvider.UrlCallback() {
//            @Override
//            public void onUrl(@NonNull String wssUrl) {
//                live = new LiveSession(MainActivityBrainOnTablet.this);
//                audio.initPlayback();
//                live.connect(wssUrl);
//            }
//            @Override
//            public void onError(@NonNull String message) {
//                runOnUiThread(() -> setStatus("Auth error: " + message));
//            }
//        });
//    }
//
//    private void stopSession() {
//        running = false;
//        audio.stopCapture();
//        audio.releasePlayback();
//        if (live != null) live.close();
//        runOnUiThread(() -> {
//            startStopButton.setText("Start");
//            setStatus("Stopped.");
//        });
//    }
//
//    /** Call this from your camera source at ~1 fps to give Gemini visual context. */
//    public void pushFrame(@NonNull byte[] jpeg) {
//        if (live != null && running) live.sendVideoFrame(jpeg);
//        Bitmap bmp = BitmapFactory.decodeByteArray(jpeg, 0, jpeg.length);
//        if (bmp != null) runOnUiThread(() -> frameView.setImageBitmap(bmp));
//    }
//
//    // ---- LiveSession.Listener ----
//
//    @Override
//    public void onReady() {
//        running = true;
//        audio.startCapture((data, len) -> { if (live != null) live.sendAudio(data, len); });
//        runOnUiThread(() -> {
//            startStopButton.setText("Stop");
//            setStatus("Listening. Speak to the robot anytime.");
//        });
//    }
//
//    @Override
//    public void onAudioOutput(@NonNull byte[] pcm24k) {
//        audio.playChunk(pcm24k);
//    }
//
//    @Override
//    public void onTranscript(@NonNull String text, boolean fromUser) {
//        runOnUiThread(() -> {
//            setStatus(spokenMessage.toString());
//            spokenMessage.append(text);
//        });
//    }
//
//    @Override
//    public void onToolCall(@NonNull String name, @NonNull String id, @NonNull JSONObject args) {
//        // The model wants the arm to act. Forward to the kit, then ack the model.
//        runOnUiThread(() -> spokenMessage.setLength(0));
//        kit.sendCommand(name + " " + args.toString());
//        if (live != null) {
//            try {
//                live.sendToolResponse(name, id,
//                        new JSONObject().put("status", "dispatched"));
//            } catch (Exception ignored) {}
//        }
//    }
//
//    @Override
//    public void onInterrupted() {
//        audio.flushPlayback();
//        spokenMessage.setLength(0);
//    }
//
//    @Override
//    public void onClosed(@NonNull String reason) {
//        runOnUiThread(() -> setStatus("Session closed: " + reason));
//    }
//
//    @Override
//    public void onError(@NonNull String message) {
//        runOnUiThread(() -> setStatus("Live error: " + message));
//    }
//
//    // ---- KitLink.Listener ----
//
//    @Override public void onKitStatus(@NonNull String status) {
//        runOnUiThread(() -> setStatus("Kit: " + status));
//    }
//    @Override public void onKitFrame(@NonNull byte[] jpeg) { pushFrame(jpeg); }
//    @Override public void onKitConnectionChanged(boolean connected, @NonNull String detail) {
//        runOnUiThread(() -> toast(detail));
//    }
//
//    // ---- plumbing ----
//
//    private void setStatus(@NonNull String text) { statusView.setText(text); }
//    private void toast(@NonNull String msg) { Toast.makeText(this, msg, Toast.LENGTH_SHORT).show(); }
//
//    private void requestRuntimePermissions() {
//        java.util.ArrayList<String> needed = new java.util.ArrayList<>();
//        if (checkSelfPermission(Manifest.permission.RECORD_AUDIO)
//                != PackageManager.PERMISSION_GRANTED) {
//            needed.add(Manifest.permission.RECORD_AUDIO);
//        }
//        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S
//                && checkSelfPermission(Manifest.permission.BLUETOOTH_CONNECT)
//                != PackageManager.PERMISSION_GRANTED) {
//            needed.add(Manifest.permission.BLUETOOTH_CONNECT);
//        }
//        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S
//                && checkSelfPermission(Manifest.permission.BLUETOOTH_SCAN)
//                != PackageManager.PERMISSION_GRANTED) {
//            needed.add(Manifest.permission.BLUETOOTH_SCAN);
//        }
//        if (!needed.isEmpty()) {
//            ActivityCompat.requestPermissions(this, needed.toArray(new String[0]), 1001);
//        }
//    }
//
//    @Override
//    protected void onDestroy() {
//        super.onDestroy();
//        stopSession();
//        if (kit != null) kit.disconnect();
//    }
//}
