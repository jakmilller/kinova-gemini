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
import android.widget.RadioButton;
import android.widget.RadioGroup;
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
    private Spinner deviceSpinner;
    private TextView btHint;
    private final List<String> deviceLabels = new ArrayList<>();
    private final List<String> deviceMacs = new ArrayList<>();
    private AppSettings settings;
    private RadioGroup transportGroup;
    private RadioButton radioTcp;
    private RadioButton radioBluetooth;
    private RadioButton radioBle;
    private View tcpSection;
    private View bluetoothSection;

    private static final Pattern IPV4 = Pattern.compile("^(\\d{1,3})\\.(\\d{1,3})\\.(\\d{1,3})\\.(\\d{1,3})\\$");

    @Override
    protected void onCreate(@Nullable Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_settings);

        settings = new AppSettings(this);
        hostField = findViewById(R.id.hostField);
        transportGroup = findViewById(R.id.transportGroup);
        radioTcp = findViewById(R.id.radioTcp);
        radioBluetooth = findViewById(R.id.radioBluetooth);
        radioBle = findViewById(R.id.radioBle);
        tcpSection = findViewById(R.id.tcpSection);
        bluetoothSection = findViewById(R.id.bluetoothSection);
        deviceSpinner = findViewById(R.id.deviceSpinner);
        btHint = findViewById(R.id.btHint);
        Button saveButton = findViewById(R.id.saveButton);

        if(settings.isBluetooth()) radioBluetooth.setChecked(true);
        else if(settings.isBle()) radioBle.setChecked(true);
        else radioTcp.setChecked(true);

        // Pre-fill existing values so editing is easy.
        String existingHost = settings.getKitHost();
        if (existingHost != null) hostField.setText(existingHost);

        showRelevantSection();
        transportGroup.setOnCheckedChangeListener((g, id) -> showRelevantSection());

        bluetoothPermissionCheck();
        loadPairedDevices();

        saveButton.setOnClickListener(v -> save());
    }

    private void showRelevantSection(){
        boolean bt = radioBluetooth.isChecked();
        boolean ble = radioBle.isChecked();
        tcpSection.setVisibility(bt | ble ? View.GONE : View.VISIBLE);
        bluetoothSection.setVisibility(bt ? View.VISIBLE : View.GONE);
    }

    private void loadPairedDevices(){
        deviceLabels.clear();
        deviceMacs.clear();

        BluetoothAdapter adapter = BluetoothAdapter.getDefaultAdapter();
        if(adapter == null){
            btHint.setText("No BT available on this device");
            return;
        }

        if(!adapter.isEnabled()){
            btHint.setText("Turn BT on, then retry");
            return;
        }

        Set<BluetoothDevice> paired = adapter.getBondedDevices();
        if(paired == null || paired.isEmpty()){
            btHint.setText("No paired devices. Pair the kit in this phone's system BT settings");
            return;
        }

        btHint.setText("Pick the robot kit from the list");
        String savedMac = settings.getKitMac();
        int savedIndex = 0;
        int i = 0;

        for (BluetoothDevice d : paired) {
            String name = d.getName() != null ? d.getName() : "unnamed";
            deviceLabels.add(name + " [" + d.getAddress() + "]");
            deviceMacs.add(d.getAddress());
            if(d.getAddress().equals(savedMac)) savedIndex = i;
            i++;
        }

        deviceSpinner.setAdapter(new ArrayAdapter<>(this, android.R.layout.simple_spinner_dropdown_item, deviceLabels));
        deviceSpinner.setSelection(savedIndex);
    }

    private void save() {

        if(radioBluetooth.isChecked()){
            if(deviceMacs.isEmpty()){
                toast("No paired devices. Pair the kit in BT settings first");
                return;
            }

            int pos = deviceSpinner.getSelectedItemPosition();
            if(pos < 0 || pos > deviceMacs.size()){
                toast("Select the kit from the list.");
                return;
            }

            settings.setKitMac(deviceMacs.get(pos));
            settings.setTransport(AppSettings.TRANSPORT_BT);
        }
        else if (radioBle.isChecked()){
            settings.setTransport(AppSettings.TRANSPORT_BLE);
        }
        else{
            String host = hostField.getText().toString().trim();

            // Validate: must be a non-empty https URL (https keeps us off cleartext).
            if (host.isEmpty()) {
                toast("Enter the kit's IP address.");
                return;
            }

            if (!looksLikeAddress(host)){
                toast("That doesn't look like a valid IP address or hostname");
                return;
            }

            settings.setKitHost(host);
            settings.setTransport(AppSettings.TRANSPORT_TCP);

        }

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

    private void bluetoothPermissionCheck(){
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S && checkSelfPermission(Manifest.permission.BLUETOOTH_CONNECT) != PackageManager.PERMISSION_GRANTED){
            ActivityCompat.requestPermissions(this, new String[]{Manifest.permission.BLUETOOTH_CONNECT}, 2001);
        }
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S && checkSelfPermission(Manifest.permission.BLUETOOTH_ADVERTISE) != PackageManager.PERMISSION_GRANTED){
            ActivityCompat.requestPermissions(this, new String[]{Manifest.permission.BLUETOOTH_ADVERTISE}, 2002);
        }
    }

    private void toast(@NonNull String msg) {
        Toast.makeText(this, msg, Toast.LENGTH_SHORT).show();
    }
}
