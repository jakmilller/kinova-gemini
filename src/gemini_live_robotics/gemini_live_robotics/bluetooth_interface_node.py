import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import ctypes
import os
import socket
import threading

RFCOMM_CHANNEL = 1

# Anaconda's CPython is built without <bluetooth/bluetooth.h>, so its socket module has no
# AF_BLUETOOTH and cannot parse an RFCOMM address tuple. The kernel (BlueZ) supports RFCOMM
# regardless, so we skip the missing Python layer and call libc directly for the three
# address-taking syscalls. Everything after accept() -- recv/sendall/close -- is
# address-agnostic and works on a plain socket object on any interpreter.
AF_BLUETOOTH = 31
BTPROTO_RFCOMM = 3

_libc = ctypes.CDLL("libc.so.6", use_errno=True)


class _SockaddrRC(ctypes.Structure):
    """struct sockaddr_rc from <bluetooth/rfcomm.h>."""
    _fields_ = [
        ("rc_family", ctypes.c_ushort),
        ("rc_bdaddr", ctypes.c_ubyte * 6),
        ("rc_channel", ctypes.c_ubyte),
    ]


def _check(ret, what):
    if ret < 0:
        err = ctypes.get_errno()
        raise OSError(err, f'{what}: {os.strerror(err)}')
    return ret


def rfcomm_listen(channel, backlog=1):
    """Bind an RFCOMM listening socket on `channel` for any local adapter (BDADDR_ANY)."""
    server = socket.socket(AF_BLUETOOTH, socket.SOCK_STREAM, BTPROTO_RFCOMM)
    try:
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        addr = _SockaddrRC(rc_family=AF_BLUETOOTH, rc_channel=channel)  # bdaddr zeroed = ANY
        _check(_libc.bind(server.fileno(), ctypes.byref(addr), ctypes.sizeof(addr)), 'bind')
        _check(_libc.listen(server.fileno(), backlog), 'listen')
    except Exception:
        server.close()
        raise
    return server


def rfcomm_accept(server):
    """Block until a peer connects; return (conn_socket, 'AA:BB:..' peer address)."""
    addr = _SockaddrRC()
    length = ctypes.c_uint(ctypes.sizeof(addr))
    # CDLL releases the GIL for the duration of the call, so this blocks the thread, not rclpy.
    fd = _check(_libc.accept(server.fileno(), ctypes.byref(addr), ctypes.byref(length)), 'accept')
    # BlueZ stores bdaddr least-significant byte first, so reverse it for display.
    peer = ':'.join(f'{b:02X}' for b in reversed(bytes(addr.rc_bdaddr)))
    return socket.socket(fileno=fd), peer

class BluetoothInterfaceNode(Node):
    """
    Tablet does speech-to-text and sends a plain text instruction over bluetooth. This node 
    receives the text and republishes to user_instructions (ala text_interface_node)
    
    Also forwards brain status strings back to the tablet

    """

    def __init__(self):
        super().__init__('bluetooth_interface_node')
        self.publisher = self.create_publisher(String, '/user_instructions', 10)
        self.create_subscription(String, '/brain_status', self.status_callback, 10)

        self.client_conn = None
        self.client_lock = threading.Lock()
        self.server = None

        self.server_thread = threading.Thread(target=self.run_server, daemon=True)
        self.server_thread.start()

        self.get_logger().info(f'Bluetooth interface node listening on RFCOMM ch {RFCOMM_CHANNEL}')

    def run_server(self):

        try:
            server = rfcomm_listen(RFCOMM_CHANNEL)
        except OSError as e:
            self.get_logger().error(f'Cannot make BT socket ({e})')
            return

        self.server = server

        while True:
            self.get_logger().info("Waiting for the app BT connection...")
            try:
                conn, addr = rfcomm_accept(server)
            except OSError as e:
                self.get_logger().error(f'BT accept failed, stopping server ({e})')
                return
            self.get_logger().info(f'Tablet BT connected: {addr}')

            with self.client_lock:
                self.client_conn = conn

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
                            self.publish_instruction(instruction)
            except OSError as e: 
                self.get_logger().warn(f'Connection ended: {e}')
            finally:
                with self.client_lock:
                    self.client_conn = None
                try:
                    conn.close()
                except Exception:
                    pass

    def status_callback(self, msg):
        self.send_to_tablet(msg.data)

    def send_to_tablet(self, text):
        with self.client_lock:
            conn = self.client_conn
        if conn is not None: 
            try: 
                conn.sendall((text + "\n"). encode("utf-8"))
            except OSError as e:
                self.get_logger().warn(f'Failed to send BT status to tablet: {e}')

    def publish_instruction(self, instruction):
        msg = String()
        msg.data = instruction
        self.publisher.publish(msg)
        self.get_logger().info(f'Published BT instruction from tablet: {instruction}')

def main(args=None):
    rclpy.init(args=args)
    node = BluetoothInterfaceNode()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    except KeyboardInterrupt:
        pass
    finally:
        # Release the RFCOMM channel promptly so an immediate relaunch can rebind it.
        for sock in (node.client_conn, node.server):
            if sock is not None:
                try:
                    sock.close()
                except OSError:
                    pass
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()