
import asyncio
import ctypes
import os
import socket
import threading

RFCOMM_CHANNEL = 4

# Anaconda's CPython is built without <bluetooth/bluetooth.h>, so its socket module has no
# AF_BLUETOOTH and cannot parse an RFCOMM address tuple. The kernel (BlueZ) supports RFCOMM
# regardless, so we skip the missing Python layer and call libc directly for the three
# address-taking syscalls. Everything after accept() -- recv/sendall/close -- is
# address-agnostic and works on a plain socket object on any interpreter.
AF_BLUETOOTH = 31
BTPROTO_RFCOMM = 3

import socket

def rfcomm_listen(channel=1, backlog=1):
    """Bind an RFCOMM listening socket natively on Windows/Linux."""
    server = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
    
    try:
        server.bind((socket.BDADDR_ANY, channel))
        
        server.listen(backlog)
        print(f"Listening for Bluetooth connections on RFCOMM channel {channel}...")
        
    except Exception:
        server.close()
        raise
        
    return server

def find_free_channel(start=1, end=30):
    for ch in range(start, end + 1):
        s = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
        try:
            s.bind((socket.BDADDR_ANY, ch))
            s.listen(1)
            print(f"Ch {ch} is free")
            return s, ch
        except OSError:
            s.close()
    raise OSError("No free RFCOMM channel, checked 1-30")

def rfcomm_accept(server):

    print("Waiting for an incoming Android connection...")
    
    conn_socket, client_info = server.accept()
    peer_address = client_info[0] 
    
    print(f"Connection accepted from remote device: {peer_address}")
    return conn_socket, peer_address

def run_server():

    try:
        server, ch = find_free_channel()
        
    except OSError as e:
        print(f'Cannot make BT socket ({e})')
        return

    while True:
        print("Waiting for the app BT connection...")
        try:
            conn, addr = rfcomm_accept(server)
        except OSError as e:
            print(f'BT accept failed, stopping server ({e})')
            return
        print(f'Tablet BT connected: {addr}')


        buf = b""
        try:
            while True:
                data = conn.recv(1024)
                if not data:
                    break
                buf += data
                while b"\n" in buf:
                    line, _, buf = buf.partition(b"\n")
                    instruction = line.decode("utf-8", errors="replace").strip()
                    if instruction:
                        print(instruction)
        except OSError as e: 
            print(f'Connection ended: {e}')
        finally:
            try:
                conn.close()
            except Exception:
                pass

def publish_instruction(instruction):
    print(f'Published BT instruction from tablet: {instruction}')

async def main():
    run_server()

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nStopped.")