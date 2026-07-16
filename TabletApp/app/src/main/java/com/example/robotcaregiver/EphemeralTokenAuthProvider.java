package com.example.robotcaregiver;

import android.provider.MediaStore;

import androidx.annotation.NonNull;

import org.json.JSONObject;

import java.io.IOException;
import java.net.URLEncoder;
import java.util.concurrent.TimeUnit;

import okhttp3.Call;
import okhttp3.Callback;
import okhttp3.MediaType;
import okhttp3.OkHttpClient;
import okhttp3.Request;
import okhttp3.RequestBody;
import okhttp3.Response;

/**
 * to be implemented later
 *
 * The model:
 *   - A small backend holds the real Gemini API key instead of the app.
 *   - The app calls your backend over HTTPS and asks for a session.
 *   - The backend mints a token from the Gemini API and returns it and the URL
 *     to the app.
 *   - App opens the Live WebSocket using that token instead of the raw key.
 *     If the token leaks, it expires soon anyway.
 *
 */
public class EphemeralTokenAuthProvider implements com.example.robotcaregiver.live.LiveAuthProvider {

    private final String backendBaseUrl;
    private static final String WSS_BASE = "wss://generativelanguage.googleapis.com/ws/" +
                                            "google.ai.generativelanguage.v1alpha.GenerativeService.BidiGenerateContentConstrained";
    private static final MediaType JSON = MediaType.parse("apllication/json; charset=utf-8");
    private final String appAuthHeader;
    private final String appAuthValue;
    private final OkHttpClient http;

    public EphemeralTokenAuthProvider(@NonNull String backendBaseUrl,
                                      @NonNull String appAuthHeader,
                                      @NonNull String appAuthValue) {
        this.backendBaseUrl = backendBaseUrl;
        this.appAuthHeader = appAuthHeader;
        this.appAuthValue = appAuthValue;

        this.http = new OkHttpClient.Builder().callTimeout(10, TimeUnit.SECONDS).build();
    }

    @Override
    public void getConnectionUrl(@NonNull UrlCallback cb) {

        Request request = new Request.Builder()
                    .url(backendBaseUrl)
                    .addHeader(appAuthHeader, appAuthValue)
                    .post(RequestBody.create("", JSON))
                    .build();

        http.newCall(request).enqueue(new Callback() {
            @Override
            public void onFailure(@NonNull Call call, @NonNull IOException e) {
                cb.onError("Token fetch failed: " + e.getMessage());
            }

            @Override
            public void onResponse(@NonNull Call call, @NonNull Response response) throws IOException {
                try {
                    String body = (response.body() != null) ? response.body().string() : "";
                    if(!response.isSuccessful()){
                        cb.onError("Token endpoint HTTP " + response.code() + ": " + body);
                        return;
                    }

                    String tokenName = extractTokenName(body);
                    if(tokenName == null || tokenName.isEmpty()){
                        cb.onError("Token endpoint returned no token name: " + body);
                        return;
                    }

                    String encoded = URLEncoder.encode(tokenName, "UTF-8");
                    cb.onUrl(WSS_BASE + "?access_token=" + encoded);
                } catch (Exception e){
                    cb.onError("Token parse error: " + e.getMessage());
                }
            }
        });
    }

    private String extractTokenName(@NonNull String json) throws Exception {
        JSONObject obj = new JSONObject(json);
        if(obj.has("token")) return obj.getString("token");
        if(obj.has("name")) return obj.getString("name");
        if(obj.has("error")){
            throw new IllegalStateException("Backend error: " + obj.getString("error"));
        }

        return null;
    }

    @Override
    public boolean isProductionSafe() {
        return true;
    }
}
