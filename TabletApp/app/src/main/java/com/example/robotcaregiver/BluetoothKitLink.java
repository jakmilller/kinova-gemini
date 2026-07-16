package com.example.robotcaregiver;

import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothSocket;
import android.util.Log;

import androidx.annotation.NonNull;

import java.io.InputStream;
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

    private final String deviceMac;
    private Listener listener;
    private BluetoothSocket socket;
    private OutputStream out;
    private volatile boolean running;

    public BluetoothKitLink(@NonNull String deviceMac) { this.deviceMac = deviceMac; }

    @Override public void setListener(Listener listener) { this.listener = listener; }

    @Override
    @SuppressWarnings("MissingPermission")
    public void connect() {
        new Thread(() -> {
            try {
                BluetoothAdapter adapter = BluetoothAdapter.getDefaultAdapter();
                if (adapter == null) { notifyConn(false, "No Bluetooth adapter"); return; }
                BluetoothDevice device = adapter.getRemoteDevice(deviceMac);
                socket = device.createRfcommSocketToServiceRecord(SPP_UUID);
                adapter.cancelDiscovery();
                socket.connect();
                out = socket.getOutputStream();
                running = true;
                notifyConn(true, "Connected to kit " + deviceMac);
                readLoop(socket.getInputStream());
            } catch (Exception e) {
                try {
                    socket = (BluetoothSocket)deviceMac.getClass()
                            .getMethod("createRfcommSocket", new Class[] {int.class})
                            .invoke(deviceMac, 1);
                    socket.connect();
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

    private void readLoop(InputStream in) {
        byte[] buf = new byte[1024];
        StringBuilder line = new StringBuilder();
        try {
            while (running) {
                int n = in.read(buf);
                if (n < 0) break;
                String chunk = new String(buf, 0, n, StandardCharsets.UTF_8);
                for (char c : chunk.toCharArray()) {
                    if (c == '\n') {
                        if (listener != null) listener.onKitStatus(line.toString());
                        line.setLength(0);
                    } else line.append(c);
                }
            }
        } catch (Exception e) {
            if (running) notifyConn(false, "Read loop ended: " + e.getMessage());
        }
    }

    @Override
    public void sendCommand(@NonNull String command) {
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
