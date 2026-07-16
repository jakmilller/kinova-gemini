package com.example.robotcaregiver;

import androidx.annotation.NonNull;

/**
 * for dev prototype: authenticate the Live WebSocket with the raw API
 * key as a parameter from BuildConfig
 *
 */
public class RawKeyAuthProvider implements com.example.robotcaregiver.live.LiveAuthProvider {

    private static final String WSS_BASE =
            "wss://generativelanguage.googleapis.com/ws/"
          + "google.ai.generativelanguage.v1beta.GenerativeService.BidiGenerateContent";

    private final String apiKey;

    public RawKeyAuthProvider(@NonNull String apiKey) {
        this.apiKey = apiKey;
    }

    @Override
    public void getConnectionUrl(@NonNull UrlCallback cb) {
        if (apiKey == null || apiKey.isEmpty()) {
            cb.onError("No API key. Put GEMINI_API_KEY in local.properties.");
            return;
        }
        // Option 1 is synchronous: no network round-trip to mint a token.
        cb.onUrl(WSS_BASE + "?key=" + apiKey);
    }

    @Override
    public boolean isProductionSafe() {
        return false; // raw key in APK -- bench only
    }
}
