package com.example.robotcaregiver;

import android.Manifest;
import android.app.Activity;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothManager;
import android.bluetooth.BluetoothSocket;
import android.content.Context;
import android.content.pm.PackageManager;
import android.os.Build;
import android.util.Log;

import androidx.annotation.NonNull;
import androidx.core.app.ActivityCompat;

import java.io.BufferedReader;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.OutputStream;
import java.nio.charset.StandardCharsets;
import java.util.UUID;

/**
 * BTC link to the kit.
 *
 * Opens an RFCOMM socket to a paired device and runs a read loop that displays
 * line as kit status. Device MAC is currently hardcoded in.
 */
public class BluetoothKitLink implements KitLink {

    private static final String TAG = "BluetoothKitLink";
    private static final UUID SPP_UUID =
            UUID.fromString("00001101-0000-1000-8000-00805F9B34FB");
    private static final int RFCOMM_CHANNEL = 1;

    private final String deviceMac;
    private Listener listener;
    private BluetoothSocket socket;
    private OutputStream out;
    private final Context context;
    private volatile boolean running;

    public BluetoothKitLink(@NonNull String deviceMac, Context context) { this.deviceMac = deviceMac; this.context = context; }

    @Override public void setListener(Listener listener) { this.listener = listener; }

    @Override
    @SuppressWarnings("MissingPermission")
    public void connect() {
        new Thread(() -> {
            try {

                BluetoothManager manager = (BluetoothManager) context.getSystemService(Context.BLUETOOTH_SERVICE);

                if(manager == null){
//                    notifyDbg("ble manager null!");
                    notifyConn(false, "No BLE manager");
                    return;
                }
//                notifyDbg("retrieved ble manager");

                BluetoothAdapter adapter = manager.getAdapter();
                if(adapter == null || !adapter.isEnabled()){
//                    notifyDbg("ble adapter null!");
                    notifyConn(false, "Bluetooth is off. Turn it on and retry.");
                    return;
                }
//                notifyDbg("retrieved ble adapter");

                if (ActivityCompat.checkSelfPermission(context, Manifest.permission.BLUETOOTH_SCAN) != PackageManager.PERMISSION_GRANTED) {
//                    notifyDbg("need to grant advertise permission");
                    ActivityCompat.requestPermissions((Activity) context, new String[]{Manifest.permission.BLUETOOTH_SCAN}, 2002);
                    return;
                }

                if (ActivityCompat.checkSelfPermission(context, Manifest.permission.BLUETOOTH_CONNECT) != PackageManager.PERMISSION_GRANTED) {
//                    notifyDbg("need to grant ble connect permission");
                    ActivityCompat.requestPermissions((Activity) context, new String[]{Manifest.permission.BLUETOOTH_CONNECT}, 2001);
                    return;
                }

                if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S){
                    boolean scanOk = context.checkSelfPermission(Manifest.permission.BLUETOOTH_SCAN) == PackageManager.PERMISSION_GRANTED;
                    boolean connectedOk = context.checkSelfPermission(Manifest.permission.BLUETOOTH_CONNECT) == PackageManager.PERMISSION_GRANTED;

                    if(!scanOk || !connectedOk) notifyConn(false, "Missing bluetooth permission" + (scanOk ? "" : " SCAN") + (connectedOk ? "" : " CONNECT") + ". Grant permissions in app settings and try again.");
                }


                BluetoothDevice device = adapter.getRemoteDevice(deviceMac);
                socket = device.createRfcommSocketToServiceRecord(SPP_UUID);
                adapter.cancelDiscovery();
                socket.connect();
                out = socket.getOutputStream();
                running = true;
                notifyConn(true, "Connected to kit " + deviceMac);
                readLoop(socket);
            } catch (Exception e) {
                try {
                    BluetoothManager manager = (BluetoothManager) context.getSystemService(Context.BLUETOOTH_SERVICE);
                    BluetoothAdapter adapter = manager.getAdapter();
                    BluetoothDevice device = adapter.getRemoteDevice(deviceMac);
                    socket = (BluetoothSocket)device.getClass()
                            .getMethod("createRfcommSocket", new Class[] {int.class})
                            .invoke(device, 4);
                    socket.connect();
                    out = socket.getOutputStream();
                    running = true;
                    notifyConn(true, "Connected to kit " + deviceMac);
                    readLoop(socket);
                }
                catch (Exception e2){
                    Log.e(TAG, "connect failed", e);
                    notifyConn(false, "Connect failed: " + e.getMessage());
                    Log.e(TAG, "connect failed", e2);
                    notifyConn(false, "Connect failed: " + e2.getMessage());
                }
            }
        }, "kit-bt-connect").start();
    }

    private void readLoop(BluetoothSocket s) {
        try {
            BufferedReader br = new BufferedReader(new InputStreamReader(s.getInputStream(), StandardCharsets.UTF_8));
            String line;
            while (running && (line = br.readLine()) != null) {
                if(listener != null && !line.isEmpty()){
                    listener.onKitStatus(line);
                }
            }
        } catch (Exception e) {
            if (running) notifyConn(false, "Read loop ended: " + e.getMessage());
        }
    }

    @Override
    public void sendInstruction(@NonNull String command) {
        new Thread(() -> {
            try {
                if (out != null) {
                    out.write((command + "\n").getBytes(StandardCharsets.UTF_8));
                    out.flush();
                }
            } catch (Exception e) {
                Log.e(TAG, "send failed", e);
                notifyConn(false, "Send failed: " + e.getMessage());
            }
        }, "kit-bt-send").start();
    }

    @Override
    public void disconnect() {
        running = false;
        try { if (socket != null) socket.close(); } catch (Exception ignored) {}
        notifyConn(false, "Disconnected");
    }

    @Override public boolean isConnected() { return socket != null && socket.isConnected(); }

    private void notifyConn(boolean connected, String detail) {
        if (listener != null) listener.onKitConnectionChanged(connected, detail);
    }
}
