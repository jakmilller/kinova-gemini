package com.example.robotcaregiver;

import android.Manifest;
import android.app.Activity;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothGatt;
import android.bluetooth.BluetoothGattCharacteristic;
import android.bluetooth.BluetoothGattDescriptor;
import android.bluetooth.BluetoothGattServer;
import android.bluetooth.BluetoothGattServerCallback;
import android.bluetooth.BluetoothGattService;
import android.bluetooth.BluetoothManager;
import android.bluetooth.BluetoothProfile;
import android.bluetooth.le.AdvertiseCallback;
import android.bluetooth.le.AdvertiseData;
import android.bluetooth.le.AdvertiseSettings;
import android.bluetooth.le.BluetoothLeAdvertiser;
import android.content.Context;
import android.content.pm.PackageManager;
import android.os.Build;
import android.os.ParcelUuid;
import android.util.Log;

import androidx.annotation.NonNull;
import androidx.core.app.ActivityCompat;

import java.nio.charset.StandardCharsets;
import java.util.UUID;

/**
 * BLE link to the kit.
 *
 * Phone advertises GATT server, robot kit scans for it and connects
 */

public class BleKitLink implements KitLink {
    private static final String TAG = "BleKitLink";
    private static final UUID SERVICE_UUID = UUID.fromString("6e400001-b5a3-f393-e0a9-e50e24dcca9e");
    private static final UUID INSTRUCTION_UUID = UUID.fromString("6e400002-b5a3-f393-e0a9-e50e24dcca9e");
    private static final UUID STATUS_UUID = UUID.fromString("6e400003-b5a3-f393-e0a9-e50e24dcca9e");
    private static final UUID CCCD_UUID = UUID.fromString("00002902-0000-1000-8000-00805f9b34fb");

    private final Context context;
    private Listener listener;

    private BluetoothGattServer gattServer;
    private BluetoothLeAdvertiser advertiser;
    private BluetoothGattCharacteristic instructionChar;
    private BluetoothGattCharacteristic statusChar;
    private BluetoothDevice connectedCentral;
    private boolean notificationsEnabled;
    private final ImbFrame frame = new ImbFrame();

    public BleKitLink(Context context){
        this.context = context;
    }

    @Override
    public void setListener(Listener listener){
        this.listener = listener;
    }

    @Override
    public void connect(){
        try{
            connectInternal();
        }
        catch (SecurityException e){
            notifyConn(false, "Bluetooth permission denied: " + e.getMessage());
        }
        catch (Exception e){
            notifyConn(false, "Couldn't start BLE: " + e.getMessage());
        }
    }

    public void connectInternal(){
        BluetoothManager manager = (BluetoothManager) context.getSystemService(Context.BLUETOOTH_SERVICE);

        if(manager == null){
            notifyDbg("ble manager null!");
            notifyConn(false, "No BLE manager");
            return;
        }
        notifyDbg("retrieved ble manager");

        BluetoothAdapter adapter = manager.getAdapter();
        if(adapter == null || !adapter.isEnabled()){
            notifyDbg("ble adapter null!");
            notifyConn(false, "Bluetooth is off. Turn it on and retry.");
            return;
        }
        notifyDbg("retrieved ble adapter");

        if(!adapter.isMultipleAdvertisementSupported()){
            Log.w(TAG, "no multi-advertisement supported");
        }

        if (ActivityCompat.checkSelfPermission(context, Manifest.permission.BLUETOOTH_ADVERTISE) != PackageManager.PERMISSION_GRANTED) {
            notifyDbg("need to grant advertise permission");
            ActivityCompat.requestPermissions((Activity) context, new String[]{Manifest.permission.BLUETOOTH_ADVERTISE}, 2002);
            return;
        }

        if (ActivityCompat.checkSelfPermission(context, Manifest.permission.BLUETOOTH_CONNECT) != PackageManager.PERMISSION_GRANTED) {
            notifyDbg("need to grant ble connect permission");
            ActivityCompat.requestPermissions((Activity) context, new String[]{Manifest.permission.BLUETOOTH_CONNECT}, 2001);
            return;
        }

        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S){
            boolean advertiseOk = context.checkSelfPermission(Manifest.permission.BLUETOOTH_ADVERTISE) == PackageManager.PERMISSION_GRANTED;
            boolean connectedOk = context.checkSelfPermission(Manifest.permission.BLUETOOTH_CONNECT) == PackageManager.PERMISSION_GRANTED;

            if(!advertiseOk || !connectedOk) notifyConn(false, "Missing bluetooth permission" + (advertiseOk ? "" : " ADVERTISE") + (connectedOk ? "" : " CONNECT") + ". Grant permissions in app settings and try again.");
        }

        gattServer = manager.openGattServer(context, serverCallback);
        if(gattServer == null){
            notifyConn(false, "Could not open gatt server");
            notifyDbg("gattServer null!");
            return;
        }
        notifyDbg("retrieved gattServer");

        BluetoothGattService service = new BluetoothGattService(SERVICE_UUID, BluetoothGattService.SERVICE_TYPE_PRIMARY);

        notifyDbg("created ble service");

        instructionChar = new BluetoothGattCharacteristic(INSTRUCTION_UUID, BluetoothGattCharacteristic.PROPERTY_NOTIFY, BluetoothGattCharacteristic.PERMISSION_READ);
        BluetoothGattDescriptor cccd = new BluetoothGattDescriptor(CCCD_UUID, BluetoothGattDescriptor.PERMISSION_READ | BluetoothGattDescriptor.PERMISSION_WRITE);
        instructionChar.addDescriptor(cccd);
        service.addCharacteristic(instructionChar);

        notifyDbg("created instruction characteristic");

        statusChar = new BluetoothGattCharacteristic(STATUS_UUID, BluetoothGattCharacteristic.PROPERTY_WRITE | BluetoothGattCharacteristic.PROPERTY_WRITE_NO_RESPONSE, BluetoothGattCharacteristic.PERMISSION_WRITE);
        service.addCharacteristic(statusChar);

        notifyDbg("created status characteristic");

        gattServer.addService(service);

        notifyDbg("added service to gattServer");

        advertiser = adapter.getBluetoothLeAdvertiser();
        if(advertiser == null){
            notifyDbg("ble advertiser null!");
            notifyConn(false, "This device can't advertise over BLE");
            return;
        }
        notifyDbg("retrieved ble advertiser");

        AdvertiseData advertiseData = new AdvertiseData.Builder()
                .setIncludeDeviceName(false)
                .addServiceUuid(new ParcelUuid(SERVICE_UUID))
                .build();

        notifyDbg("added service to advertiser");

        AdvertiseSettings settings = new AdvertiseSettings.Builder()
                .setAdvertiseMode(AdvertiseSettings.ADVERTISE_MODE_LOW_LATENCY)
                .setTxPowerLevel(AdvertiseSettings.ADVERTISE_TX_POWER_HIGH)
                .setConnectable(true)
                .setTimeout(0)
                .build();

        notifyDbg("set advertising latency & power");

        AdvertiseData scanResponse = new AdvertiseData.Builder()
                .setIncludeDeviceName(true)
                .build();

        notifyDbg("set advertising dev name");

        advertiser.startAdvertising(settings, advertiseData, scanResponse, advertiseCallback);
        notifyDbg("started advertising!");
    }

    private final AdvertiseCallback advertiseCallback = new AdvertiseCallback() {
        @Override
        public void onStartSuccess(AdvertiseSettings settingsInEffect) {
            notifyConn(false, "Advertising. Wait for kit to connect");

        }

        @Override
        public void onStartFailure(int errorCode){
            String reason;
            switch (errorCode){
                case ADVERTISE_FAILED_DATA_TOO_LARGE -> reason = "advertise data too large";
                case ADVERTISE_FAILED_TOO_MANY_ADVERTISERS -> reason = "too many advertisers";
                case ADVERTISE_FAILED_ALREADY_STARTED -> reason = "already advertising";
                case ADVERTISE_FAILED_INTERNAL_ERROR -> reason = "internal error";
                case ADVERTISE_FAILED_FEATURE_UNSUPPORTED -> reason = "BLE advertising unsupported on this device";
                default -> reason = "code " + errorCode;
            }

            Log.e(TAG, "advertising failed: " + reason);
            notifyConn(false, "advertising failed: " + reason);
        }
    };

    private final BluetoothGattServerCallback serverCallback = new BluetoothGattServerCallback() {

        @Override
        public void onDescriptorReadRequest(BluetoothDevice device, int requestId, int offset, BluetoothGattDescriptor descriptor){
            byte[] value = CCCD_UUID.equals(descriptor.getUuid()) ? (notificationsEnabled ? BluetoothGattDescriptor.ENABLE_NOTIFICATION_VALUE : BluetoothGattDescriptor.DISABLE_NOTIFICATION_VALUE) : new byte[0];

            if(gattServer != null){
                gattServer.sendResponse(device, requestId, BluetoothGatt.GATT_SUCCESS, offset, value);
            }
        }

        @Override
        public void onCharacteristicReadRequest(BluetoothDevice device, int requestId, int offset, BluetoothGattCharacteristic characteristic){

            if(gattServer != null){
                gattServer.sendResponse(device, requestId, BluetoothGatt.GATT_SUCCESS, offset, new byte[0]);
            }
        }

        @Override
        public void onMtuChanged(BluetoothDevice device, int mtu){
            Log.d(TAG, "mtu negotiated: " + mtu);
        }

        @Override
        public void onDescriptorWriteRequest(BluetoothDevice device, int requestId, BluetoothGattDescriptor descriptor, boolean preparedWrite, boolean responseNeeded, int offset, byte[] value) {

            if(CCCD_UUID.equals(descriptor.getUuid())){
                notificationsEnabled = value.length > 0 && value[0] == BluetoothGattDescriptor.ENABLE_NOTIFICATION_VALUE[0];
            }

            if(responseNeeded && gattServer != null){
                gattServer.sendResponse(device, requestId, BluetoothGatt.GATT_SUCCESS, offset, null);
            }
        }

        @Override
        public void onConnectionStateChange(BluetoothDevice device, int status, int newState) {
            if(newState == BluetoothProfile.STATE_CONNECTED){
                connectedCentral = device;
                notifyConn(true, "Kit connected");
            }
            else if(newState == BluetoothProfile.STATE_DISCONNECTED){
                connectedCentral = null;
                notificationsEnabled = false;
                frame.reset();
                Log.d(TAG, "Kit disconnected, status: " + status);
                notifyConn(false, "Kit disconnected, status: " + status);
            }
        }

        @Override
        public void onCharacteristicWriteRequest(BluetoothDevice device, int requestId, BluetoothGattCharacteristic characteristic, boolean preparedWrite, boolean responseNeeded, int offset, byte[] value) {
            if(STATUS_UUID.equals(characteristic.getUuid())){
                String complete = frame.accept(value);
                if(complete != null && listener != null && !complete.isEmpty()){
                    listener.onKitStatus(complete);
                }
            }
            if(responseNeeded && gattServer != null){
                gattServer.sendResponse(device, requestId, BluetoothGatt.GATT_SUCCESS, offset, null);
            }
        }
    };

    @Override
    public void sendInstruction(@NonNull String text){
        if(gattServer == null || connectedCentral == null || instructionChar == null){
            notifyConn(false, "Kit not connected, can't send");
            return;
        }

        if(!notificationsEnabled){
            notifyConn(false, "Kit not subscribed yet");
            return;
        }

        byte[] payload = (text + "\n").getBytes(StandardCharsets.UTF_8);
        instructionChar.setValue(payload);
        boolean ok = gattServer.notifyCharacteristicChanged(connectedCentral, instructionChar, false);
        if(!ok){
            notifyConn(false, "Failed to send instruction to kit");
        }
    }

    @Override
    public void disconnect(){
        if(advertiser != null){
            try {advertiser.stopAdvertising(advertiseCallback); } catch(Exception ignore) {}
            advertiser = null;
        }

        if(gattServer != null){
            try {gattServer.close(); } catch(Exception ignore) {}
            gattServer = null;
        }

        connectedCentral = null;
        notificationsEnabled = false;
        frame.reset();
        notifyConn(false, "Stopped advertising");
    }

    @Override
    public boolean isConnected(){
        return connectedCentral != null;
    }

    private void notifyConn(boolean connected, String msg){
        if (listener != null) listener.onKitConnectionChanged(connected, msg);
    }

    private void notifyDbg(String msg){
        if (listener != null) listener.onDbgMsg(msg);
    }

}
