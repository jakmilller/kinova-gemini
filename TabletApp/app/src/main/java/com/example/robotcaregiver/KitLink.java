package com.example.robotcaregiver;

import androidx.annotation.NonNull;

/**
 * Link between the tablet and the robot kit.
 *
 * FOr now: tablet runs the Live session and sends arm
 *   actions to the kit.
 *
 */
public interface KitLink {

    interface Listener {
        void onKitStatus(@NonNull String status);
        void onKitFrame(@NonNull byte[] jpeg);
        void onKitConnectionChanged(boolean connected, @NonNull String detail);
    }

    void connect();
    void disconnect();
    void sendCommand(@NonNull String command);
    void setListener(Listener listener);
    boolean isConnected();
}
