import rclpy
from clpy.node import Node
from std_msgs.msg import String 
import threading
import socket
import threading

LISTEN_PORT = 9100

class NetworkInterfaceNode(Node):
    """
    Tablet does speech-to-text and sends a plain text instruction over TCP socket. This node 
    receives the text and republishes to user_instructions (ala text_interface_node)
    
    Also forwards brain status strings back to the tablet

    Tablet and kit must be on the same Wi-Fi network
    """

    def __init__(self):
        super().__init__('network_interface_node')
        self.publisher = self.create_publisher(String, '/user_instructions', 10)
        self.create_subscription(String, '/brain_status', self.status_callback, 10)
        self.timer = self.create_timer(0.1, self.run_interface)
        self.get_logger().info('Network Interface Node listening...')

        self.client_conn = None
        self.client_lock = threading.Lock()

        self.server_thread = threading.Thread(target=self.run_server, daemon=True)
        self.server_thread.start()

    def run_server(self):
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind(("0.0.0.0", LISTEN_PORT))
        server.listen(1)

        while True:
            self.get_logger().info("Waiting for the app connection...")
            conn, addr = server.accept()
            self.get_logger().info(f'Tablet connected: {addr}')

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
                self.get_logger().warn(f'Failed to send status to tablet: {e}')

    def publish_instruction(self, instruction):
        msg = String()
        msg.data = instruction
        self.publisher.publish(msg)
        self.get_logger().info(f'Published instruction from tablet: {instruction}')

    def main(args=None):
        rclpy.init(args=args)
        node = NetworkInterfaceNode()
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