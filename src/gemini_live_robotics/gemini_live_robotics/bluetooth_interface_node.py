import rclpy
from rclpy.node import Node
from std_msgs.msg import String 
import socket
import threading

RFCOMM_CHANNEL = 1

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

        self.server_thread = threading.Thread(target=self.run_server, daemon=True)
        self.server_thread.start()

        self.get_logger().info(f'Bluetooth interface node listening on RFCOMM ch {RFCOMM_CHANNEL}')

    def run_server(self):

        try:
            server = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)

        except (AttributeError, OSError) as e:
            self.get_logger().error(f'Cannot make BT socket ({e})')
            return

        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind(("", RFCOMM_CHANNEL))
        server.listen(1)

        while True:
            self.get_logger().info("Waiting for the app BT connection...")
            conn, addr = server.accept()
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
            node.destroy_node()
            rclpy.shutdown()

    if __name__ == '__main__':
        main()