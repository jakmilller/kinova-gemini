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
        void onKitConnectionChanged(boolean connected, @NonNull String detail);

        void onDbgMsg(String msg);
    }

    void connect();
    void disconnect();
    void sendInstruction(@NonNull String command);
    void setListener(Listener listener);
    boolean isConnected();
}
