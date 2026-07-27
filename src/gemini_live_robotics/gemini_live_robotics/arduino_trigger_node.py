#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import serial
import serial.tools.list_ports
import time
import threading

# Push-to-talk hardware button. An old e-stop / aux button wired to an Arduino, which
# streams its state over USB serial: "1" while the button is HELD, "0" while RELEASED
# (see src/arduino/voice_button/voice_button.ino). This node converts that continuous
# level into two discrete edges and publishes them as /voice_trigger (Bool):
#   0 -> 1  (press)   => publish True  -> voice_interface_node starts recording
#   1 -> 0  (release) => publish False -> voice_interface_node stops + transcribes


class ArduinoTriggerNode(Node):
    def __init__(self):
        super().__init__('arduino_trigger_node')

        # --- Parameters ---
        self.declare_parameter('port', '')  # Auto-detect if empty
        self.declare_parameter('baudrate', 115200)

        self.port_param = self.get_parameter('port').get_parameter_value().string_value
        self.baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value

        # --- State ---
        self.serial_connection = None
        self.running = True
        self.is_pressed = False  # last published button state

        # --- ROS 2 Communication ---
        self.trigger_pub = self.create_publisher(Bool, '/voice_trigger', 10)

        # --- Start Thread ---
        self.monitor_thread = threading.Thread(target=self.monitor_arduino, daemon=True)
        self.monitor_thread.start()

        self.get_logger().info("Arduino Trigger Node Initialized (Hold-to-Talk Mode)")

    def find_arduino(self):
        """Find Arduino port automatically"""
        ports = serial.tools.list_ports.comports()

        # Try Arduino-specific identifiers
        for port in ports:
            if 'arduino' in port.description.lower() or 'micro' in port.description.lower():
                return port.device
            if port.vid is not None and (port.vid == 0x2341 or port.vid == 0x239A):  # Arduino or Adafruit
                return port.device

        # Try common Linux serial ports as fallback
        for port_name in ['/dev/ttyACM0', '/dev/ttyACM1', '/dev/ttyUSB0', '/dev/ttyUSB1']:
            try:
                test = serial.Serial(port_name, self.baudrate, timeout=0.1)
                test.close()
                return port_name
            except Exception:
                pass
        return None

    def monitor_arduino(self):
        """Serial monitoring loop"""
        while self.running:
            try:
                if not self.serial_connection:
                    target_port = self.port_param if self.port_param else self.find_arduino()
                    if target_port:
                        self.get_logger().info(f"Connecting to Arduino on {target_port}...")
                        self.serial_connection = serial.Serial(target_port, self.baudrate, timeout=0.1)
                        self.get_logger().info(f"Connected to {target_port}")
                    else:
                        time.sleep(2)
                        continue

                if self.serial_connection.in_waiting > 0:
                    line = self.serial_connection.readline().decode('utf-8', errors='ignore').strip()
                    if line in ("0", "1"):
                        self.handle_state(line == "1")

            except Exception as e:
                self.get_logger().error(f"Serial error: {e}")
                if self.serial_connection:
                    self.serial_connection.close()
                self.serial_connection = None
                time.sleep(1)

            time.sleep(0.01)

    def handle_state(self, pressed):
        """Publish only on a state change (edge), so the continuous stream becomes one
        start event and one stop event per hold."""
        if pressed == self.is_pressed:
            return
        self.is_pressed = pressed
        self.trigger_pub.publish(Bool(data=pressed))
        self.get_logger().info(f"Button {'HELD: recording STARTED' if pressed else 'RELEASED: recording STOPPED'}")


def main(args=None):
    rclpy.init(args=args)
    node = ArduinoTriggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.running = False
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
