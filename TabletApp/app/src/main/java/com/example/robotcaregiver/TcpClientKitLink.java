package com.example.robotcaregiver;

import androidx.annotation.NonNull;

import java.io.BufferedReader;
import java.io.InputStreamReader;
import java.io.OutputStream;
import java.net.InetSocketAddress;
import java.net.Socket;
import java.nio.charset.StandardCharsets;

public class TcpClientKitLink implements KitLink{


    private final String host;
    private final int port;
    private KitLink.Listener listener;
    private Socket socket;
    private OutputStream out;
    private boolean running;

    public TcpClientKitLink(String host, int port){
        this.host = host;
        this.port = port;
    }

    public void setListener(KitLink.Listener listener){ this.listener = listener; }

    public void connect() {
        new Thread(() -> {
            try {
                Socket s = new Socket();
                s.connect(new InetSocketAddress(host, port), 8000);
                socket = s;
                out = s.getOutputStream();
                running = true;
                notifyConn(true, "Connected to kit " + host + ":" + port);
                readLoop(s);
            } catch (Exception e){
                notifyConn(false, "Connection failed: " + e.getMessage());
            }
        }, "TcpClientKitConnection").start();
    }

    private void readLoop(Socket s){
        try {
            BufferedReader reader = new BufferedReader(new InputStreamReader(s.getInputStream(), StandardCharsets.UTF_8));
            String line;
            while(running && (line = reader.readLine()) != null){
                if(listener != null && !line.isEmpty()){
                    listener.onKitStatus(line);
                }
            }
        } catch (Exception e){
            if(running) notifyConn(false, "Connection lost: " + e.getMessage());
        }
    }

    public void sendInstruction(@NonNull String text){
        new Thread(() -> {
            try {
                if (out != null){
                    out.write((text + "\n").getBytes(StandardCharsets.UTF_8));
                    out.flush();
                }
            } catch (Exception e){
                notifyConn(false, "Send failed: " + e.getMessage());
            }
        }, "TcpClientKitSend").start();
    }

    public boolean isConnected() {
        return socket != null && socket.isConnected() && !socket.isClosed();
    }

    public void disconnect() {
        running = false;
        try {
            if (socket != null) socket.close();
        } catch (Exception e) {}
        notifyConn(false, "Disconnected");
    }

    private void notifyConn(boolean connected, String detail) {
        if(listener != null) listener.onKitConnectionChanged(connected, detail);
    }
}
