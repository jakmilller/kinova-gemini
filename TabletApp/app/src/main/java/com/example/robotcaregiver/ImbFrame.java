package com.example.robotcaregiver;

import androidx.annotation.NonNull;

import java.nio.charset.StandardCharsets;
import java.util.HashMap;
import java.util.Map;

public class ImbFrame {
    private final Map<Integer, byte[]> pending = new HashMap<>();
    private int expectedTotal = -1;

    public String accept(@NonNull byte[] frame){
        String asText = new String(frame, StandardCharsets.UTF_8);
        int colon = asText.indexOf(':');

        if (colon < 0){
            reset();
            return asText;
        }

        String header = asText.substring(0, colon);
        int slash = header.indexOf('/');
        if (slash < 0){
            reset();
            return asText;
        }

        int seq, total;
        try{
            seq = Integer.parseInt(header.substring(0, slash));
            total = Integer.parseInt(header.substring(slash + 1));
        }
        catch (NumberFormatException e){
            reset();
            return asText;
        }

        int headerBytes = header.getBytes(StandardCharsets.UTF_8).length + 1;
        byte[] payload = new byte[frame.length - headerBytes];
        System.arraycopy(frame, headerBytes, payload, 0, payload.length);

        if(total != expectedTotal){
            pending.clear();
            expectedTotal = total;
        }

        pending.put(seq, payload);

        if(pending.size() < expectedTotal){
            return null;
        }

        int size = 0;
        for (int i = 1; i <= expectedTotal; i++){
            byte[] p = pending.get(i);
            if(p == null) return null;
            size += p.length;
        }

        byte[] joined = new byte[size];
        int off = 0;
        for(int i = 1; i <= expectedTotal; i++){
            byte[] p = pending.get(i);
            System.arraycopy(p, 0, joined, off, p.length);
            off += p.length;
        }

        reset();
        return new String(joined, StandardCharsets.UTF_8);
    }

    public void reset(){
        pending.clear();
        expectedTotal = -1;
    }
}
