package com.example.robotcaregiver;

import android.content.Context;
import android.content.SharedPreferences;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

/**
 * Stores:
 *   - backendUrl: the full https URL of the token
 *                 (e.g. https://your-server.example.com/api/token)
 *   - kitHost:     the ip or hostname of the robot kit
 *
 * Uses SharedPreferences, which survives app restarts
 */
public class AppSettings {

    private static final String PREFS = "robot_caregiver_settings";
    private static final String KEY_BACKEND_URL = "backend_url";
    private static final String KEY_KIT_HOST = "kit_host";
    private static final String KEY_KIT_MAC = "kit_mac";
    private static final String KEY_TRANSPORT = "transport";
    public static final String TRANSPORT_TCP = "tcp";
    public static final String TRANSPORT_BT = "bluetooth";
    public static final String TRANSPORT_BLE = "ble";

    private final SharedPreferences prefs;

    public AppSettings(@NonNull Context context) {
        this.prefs = context.getApplicationContext()
                .getSharedPreferences(PREFS, Context.MODE_PRIVATE);
    }

    public String getTransport() {
        return prefs.getString(KEY_TRANSPORT, TRANSPORT_TCP);
    }

    public void setTransport(@NonNull String transport) {
        prefs.edit().putString(KEY_TRANSPORT, transport).apply();
    }

    public boolean isBluetooth() {
        return TRANSPORT_BT.equals(getTransport());
    }

    public boolean isBle(){
        return TRANSPORT_BLE.equals(getTransport());
    }

    @Nullable
    public String getBackendUrl() {
        return prefs.getString(KEY_BACKEND_URL, null);
    }

    public void setBackendUrl(@NonNull String url) {
        prefs.edit().putString(KEY_BACKEND_URL, url.trim()).apply();
    }

    @Nullable
    public String getKitHost() {
        return prefs.getString(KEY_KIT_HOST, null);
    }

    public void setKitHost(@NonNull String host) {
        prefs.edit().putString(KEY_KIT_HOST, host.trim()).apply();
    }

    @Nullable
    public String getKitMac() {
        return prefs.getString(KEY_KIT_MAC, null);
    }

    public void setKitMac(@NonNull String mac) {
        prefs.edit().putString(KEY_KIT_MAC, mac.trim()).apply();
    }

    public boolean isConfigured() {
        String url = getBackendUrl();
        String host = getKitHost();
        return url != null && !url.isEmpty() && host != null && !host.isEmpty();
    }
}
