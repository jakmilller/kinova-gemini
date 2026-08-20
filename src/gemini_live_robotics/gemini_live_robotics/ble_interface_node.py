import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import asyncio
import threading
import traceback

from .ble_link import BleCentralLink, dbg_ble

class BLEInterfaceNode(Node):
    """
    Tablet does speech-to-text and sends a plain text instruction over BLE. This node 
    receives the text and republishes to user_instructions (ala text_interface_node)
    
    Also forwards brain status strings back to the tablet

    """

    def __init__(self):
        super().__init__('ble_interface_node')
        self.publisher = self.create_publisher(String, '/user_instructions', 10)
        self.create_subscription(String, '/brain_status', self.status_callback, 10)
        self._loop = None
        self._link = BleCentralLink(self._on_instruction)
        self._thread = threading.Thread(target = self._run_ble, daemon = True)
        self._thread.start()
        self.get_logger().info("BLE Node started scanning for phone")

    def _run_ble(self):
        try:
            self._loop = asyncio.new_event_loop()
            asyncio.set_event_loop(self._loop)
            self._loop.run_until_complete(self._link.run())
        except Exception:
            dbg_ble("Fatal: BLE thread crashed: \n" + traceback.format_exc())


    def _on_instruction(self, text):
        msg = String()
        msg.data = text
        self.publisher.publish(msg)
        dbg_ble(f'Published to /user_instructions: "{text}"')
        self.get_logger().info(f'BLE instruction from phone: "{text}"')

    def status_callback(self, msg):
        dbg_ble(f'/brain_status -> phone: "{msg.data}"')
        if self._loop is None:
            dbg_ble("   dropped: BLE loop not running yet")
            return
        asyncio.run_coroutine_threadsafe(self._link.send_status(msg.data), self._loop)

    

def main(args=None):
    rclpy.init(args=args)
    node = BLEInterfaceNode()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    except KeyboardInterrupt:
        dbg_ble("Interrupted by user")
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()