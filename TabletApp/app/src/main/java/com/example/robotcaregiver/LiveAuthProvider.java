package com.example.robotcaregiver.live;

import androidx.annotation.NonNull;

/**
 * For now: RawKeyAuthProvider returns the API key from BuildConfig and
 *   builds a URL with ?key=...
 *
 * For later: EphemeralTokenAuthProvider calls backend over HTTPS, which holds the
 *   key and makes a token, it returns a ws URL/credential built from that token.
 *
 * For much later (?): the kit runs the Gemini Live session. Tablet doesn't
 *   need this at all
 */
public interface LiveAuthProvider {

    interface UrlCallback {
        void onUrl(@NonNull String wssUrl);
        void onError(@NonNull String message);
    }

    void getConnectionUrl(@NonNull UrlCallback cb);

    /**
     * Whether this provider is safe to ship in a fielded device.
     * Dev mode returns false (key in APK)
     * Ephemeral token returns true.
     */
    boolean isProductionSafe();
}
