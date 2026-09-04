package com.example.robotcaregiver;

import static android.view.View.GONE;
import static android.view.View.VISIBLE;
import static com.example.robotcaregiver.BuildConfig.GEMINI_API_KEY;

import android.Manifest;
import android.content.Intent;
import android.content.pm.PackageManager;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.os.Build;
import android.os.Bundle;
import android.os.CountDownTimer;
import android.speech.RecognizerIntent;
import android.text.method.ScrollingMovementMethod;
import android.widget.Button;
import android.widget.ImageView;
import android.widget.TextView;
import android.widget.Toast;

import androidx.activity.result.ActivityResultLauncher;
import androidx.activity.result.contract.ActivityResultContract;
import androidx.activity.result.contract.ActivityResultContracts;
import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import androidx.appcompat.app.AppCompatActivity;
import androidx.core.app.ActivityCompat;

import com.example.robotcaregiver.live.AudioIo;
import com.example.robotcaregiver.live.LiveAuthProvider;
import com.example.robotcaregiver.live.LiveSession;
import com.example.robotcaregiver.live.*;

import org.json.JSONObject;

import java.util.ArrayList;

public class MainActivity extends AppCompatActivity
        implements TcpClientKitLink.Listener {

    private TextView statusView;
    private Button startStopButton;
    private Button settingsButton;
    private Button connectButton;

    private KitLink kit;
    private AppSettings settings;
    private ActivityResultLauncher<Intent> speechLauncher;
    private ActivityResultLauncher<Intent> settingsLauncher;
    private String connectedHost;
    private static final int KIT_PORT = 9100;
    private static final long CONNECT_TIMEOUT = 30_000L;
    private CountDownTimer connectTimer;
    private String lastConnectDetail = "";

    @Override
    protected void onCreate(@Nullable Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        statusView = findViewById(R.id.statusView);
        startStopButton = findViewById(R.id.startStopButton);
        settingsButton = findViewById(R.id.settingsButton);
        connectButton = findViewById(R.id.connectButton);

        statusView.setMovementMethod(new ScrollingMovementMethod());

        settings = new AppSettings(this);

        requestRuntimePermissions();

        speechLauncher = registerForActivityResult(new ActivityResultContracts.StartActivityForResult(),
                result -> {
                    if(result.getResultCode() == RESULT_OK && result.getData() != null) {
                        ArrayList<String> matches = result.getData().getStringArrayListExtra(RecognizerIntent.EXTRA_RESULTS);
                        if(matches != null && !matches.isEmpty()) sendInstructions(matches.get(0));
                    }
                });

        settingsLauncher = registerForActivityResult(new ActivityResultContracts.StartActivityForResult(), result -> onSettingsClosed());

        startStopButton.setOnClickListener(v -> startVoiceCapture());
        settingsButton.setOnClickListener(v -> startActivity(new Intent(this, SettingsActivity.class)));
        connectButton.setOnClickListener(v -> connectToKit());

        addStatus("set click listeners to buttons");

        setConnectedUi(false);
        addStatus("Not connected to kit. Tap Connect.");
//        connectToKit();
    }

    private void onSettingsClosed() {
        String newHost = settings.getKitHost();
        if(newHost == null || newHost.isEmpty()){
            setStatus("No kit address set. Open Settings and enter the kit's IP.");
            setConnectedUi(false);
            return;
        }

        if(!newHost.equals(connectedHost) || kit == null || !kit.isConnected()){
            setStatus("Address changed. Reconnecting to " + newHost + "...");
            connectToKit();
        }
    }

    private void connectToKit(){

        addStatus("called connectToKit");

        String host = settings.getKitHost();

        addStatus("got host");

        String type = (settings.isBluetooth() ? "Bluetooth" : (settings.isBle() ? "BLE" : "WiFi"));

        if(type.equals("WiFi")){
            if(host == null || host.isEmpty()){
                addStatus("No kit address set. Open Settings and enter the kit's IP.");
                setConnectedUi(false);
                return;
            }
        }

        if(kit != null){
            kit.setListener(null);
            kit.disconnect();
            addStatus("disconnected existing kit");
        }

        connectedHost = host;

        kit = settings.isBluetooth() ? new BluetoothKitLink(settings.getKitMac(), this) : (settings.isBle() ? new BleKitLink(this) : new TcpClientKitLink(host, KIT_PORT));
        kit.setListener(this);

        addStatus("kit connection type: " + type);

        startConnectCountdown();

        kit.connect();
    }

    private void startConnectCountdown(){
        runOnUiThread(() -> {
            cancelConnectCountdown();

            connectButton.setVisibility(VISIBLE);
            connectButton.setEnabled(false);

            connectTimer = new CountDownTimer(CONNECT_TIMEOUT, 1000) {
                @Override
                public void onTick(long l) {
                    long secs = (l + 999) / 1000;
                    statusView.setText("Connection could take " + secs + " s");
                }

                @Override
                public void onFinish() {
                    connectTimer = null;
                    statusView.setText("Could not connect within 30 seconds. \nTap Connect to try again.");
                    connectButton.setVisibility(VISIBLE);
                    connectButton.setEnabled(true);

                    if (kit != null){
                        kit.setListener(null);
                        kit.disconnect();
                        kit.setListener(MainActivity.this);
                    }
                }
            }.start();
        });
    }

    private void cancelConnectCountdown(){
        if (connectTimer != null){
            connectTimer.cancel();
            connectTimer = null;
        }
    }


    private void startVoiceCapture() {
        Intent intent = new Intent(RecognizerIntent.ACTION_RECOGNIZE_SPEECH);
        intent.putExtra(RecognizerIntent.EXTRA_LANGUAGE_MODEL, RecognizerIntent.LANGUAGE_MODEL_FREE_FORM);
        intent.putExtra(RecognizerIntent.EXTRA_PROMPT, "Say your command");

        try {
            speechLauncher.launch(intent);
        } catch (Exception e){
            toast("Speech recognition unavailable: " + e.getMessage());
        }
    }

    private void sendInstructions(@NonNull String text){
        setStatus("You: " + text);
        kit.sendInstruction(text);
    }

    private void sendStatus(@NonNull String text){
        runOnUiThread(() -> statusView.setText(text));
    }

    private void setConnectedUi(boolean connected) {
        runOnUiThread(() -> {
            if(connected){
                addStatus("Connected to kit!");
                cancelConnectCountdown();
                connectButton.setVisibility(GONE);
            }
//            else if(connectTimer == null){
//                addStatus("Connection to kit lost, please reconnect.");
                connectButton.setVisibility(VISIBLE);
                connectButton.setEnabled(true);
//            }

            startStopButton.setEnabled(connected);
        });
    }


    @Override public void onKitStatus(@NonNull String status) {
        runOnUiThread(() -> { clearStatus(); setStatus("Kit: " + status); });
    }

    @Override public void onKitConnectionChanged(boolean connected, @NonNull String detail) {
        setConnectedUi(connected);
        runOnUiThread(() -> toast(detail));
    }

    @Override public void onDbgMsg(@NonNull String msg){
        runOnUiThread(() -> addStatus("Dbg: " + msg));
    }

    private void setStatus(@NonNull String text) { statusView.setText(text); }

    private void addStatus(@NonNull String text) { statusView.setText(statusView.getText().toString() + "\n" + text); }

    private void clearStatus(){ statusView.setText(""); }

    private void toast(@NonNull String msg) { Toast.makeText(this, msg, Toast.LENGTH_SHORT).show(); }

    private void requestRuntimePermissions() {
        java.util.ArrayList<String> needed = new java.util.ArrayList<>();
        if (checkSelfPermission(Manifest.permission.RECORD_AUDIO)
                != PackageManager.PERMISSION_GRANTED) {
            needed.add(Manifest.permission.RECORD_AUDIO);
        }
        if (checkSelfPermission(Manifest.permission.BLUETOOTH_ADVERTISE)
                != PackageManager.PERMISSION_GRANTED) {
            needed.add(Manifest.permission.BLUETOOTH_ADVERTISE);
        }
        if (!needed.isEmpty()) {
            ActivityCompat.requestPermissions(this, needed.toArray(new String[0]), 1001);
        }
    }

    @Override
    protected void onDestroy() {
        super.onDestroy();
        cancelConnectCountdown();
        if (kit != null) kit.disconnect();
    }
}
