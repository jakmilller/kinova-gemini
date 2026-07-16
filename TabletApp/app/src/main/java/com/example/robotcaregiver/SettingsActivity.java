package com.example.robotcaregiver;

import android.Manifest;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.content.pm.PackageManager;
import android.os.Build;
import android.os.Bundle;
import android.view.View;
import android.widget.ArrayAdapter;
import android.widget.Button;
import android.widget.EditText;
import android.widget.Spinner;
import android.widget.TextView;
import android.widget.Toast;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import androidx.appcompat.app.AppCompatActivity;
import androidx.core.app.ActivityCompat;

import java.util.ArrayList;
import java.util.List;
import java.util.Set;
import java.util.regex.Pattern;

/**
 * Settings screen: configure the app without editing code.
 *
 */
public class SettingsActivity extends AppCompatActivity {

    private EditText hostField;
    private AppSettings settings;

    private static final Pattern IPV4 = Pattern.compile("^(\\d{1,3})\\.(\\d{1,3})\\.(\\d{1,3})\\.(\\d{1,3})\\$");

    @Override
    protected void onCreate(@Nullable Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_settings);

        settings = new AppSettings(this);
        hostField = findViewById(R.id.hostField);
        Button saveButton = findViewById(R.id.saveButton);

        // Pre-fill existing values so editing is easy.
        String existingHost = settings.getKitHost();
        if (existingHost != null) hostField.setText(existingHost);

        saveButton.setOnClickListener(v -> save());
    }

    private void save() {
        String host = hostField.getText().toString().trim();

        // Validate: must be a non-empty https URL (https keeps us off cleartext).
        if (host.isEmpty()) {
            toast("Enter the kit's IP address.");
            return;
        }

        if (!looksLikeAddress(host)){
            toast("That doesn't look like a valid IP address or hostname");
        }

        settings.setKitHost(host);
        toast("Saved.");
        finish(); // return to the main screen
    }

    private boolean looksLikeAddress(@NonNull String host){
        java.util.regex.Matcher m = IPV4.matcher(host);
        if(m.matches()){
            for(int i = 1; i <= 4; i++){
                int octet = Integer.parseInt(m.group(i));
                if(octet < 0 || octet > 255) return false;
            }
            return true;
        }
        return host.matches("^[A-Za-z0-9.-]+$");
    }

    private void toast(@NonNull String msg) {
        Toast.makeText(this, msg, Toast.LENGTH_SHORT).show();
    }
}
