#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import serial
import serial.tools.list_ports
import time
import threading

# this is specific to my use case, where I adapted an old e-stop button to be used for the voice commands

class ArduinoTriggerNode(Node):
    def __init__(self):
        super().__init__('arduino_trigger_node')
        
        # --- Parameters ---
        self.declare_parameter('port', '') # Auto-detect if empty
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('debounce_window', 0.5) # Seconds to ignore rapid double-pulses
        
        self.port_param = self.get_parameter('port').get_parameter_value().string_value
        self.baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        self.debounce_window = self.get_parameter('debounce_window').get_parameter_value().double_value

        # --- State ---
        self.serial_connection = None
        self.running = True
        self.is_recording = False
        self.last_trigger_time = 0

        # --- ROS 2 Communication ---
        self.trigger_pub = self.create_publisher(Bool, '/voice_trigger', 10)

        # --- Start Thread ---
        self.monitor_thread = threading.Thread(target=self.monitor_arduino, daemon=True)
        self.monitor_thread.start()

        self.get_logger().info("Arduino Trigger Node Initialized (Toggle Mode)")

    def find_arduino(self):
        """Find Arduino port automatically"""
        ports = serial.tools.list_ports.comports()
        
        # Try Arduino-specific identifiers
        for port in ports:
            if 'arduino' in port.description.lower() or 'micro' in port.description.lower():
                return port.device
            if port.vid is not None and (port.vid == 0x2341 or port.vid == 0x239A): # Arduino or Adafruit
                return port.device

        # Try common Linux serial ports as fallback
        for port_name in ['/dev/ttyACM0', '/dev/ttyACM1', '/dev/ttyUSB0', '/dev/ttyUSB1']:
            try:
                test = serial.Serial(port_name, self.baudrate, timeout=0.1)
                test.close()
                return port_name
            except:
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
                    line = self.serial_connection.readline().decode('utf-8').strip()
                    if line == "ESTOP":
                        self.handle_trigger()

            except Exception as e:
                self.get_logger().error(f"Serial error: {e}")
                if self.serial_connection:
                    self.serial_connection.close()
                self.serial_connection = None
                time.sleep(1)
            
            time.sleep(0.01)

    def handle_trigger(self):
        """Toggle recording state with software debounce"""
        current_time = time.time()
        
        # Debounce: Ignore if triggered too soon after the last one
        if current_time - self.last_trigger_time < self.debounce_window:
            return

        self.last_trigger_time = current_time
        self.is_recording = not self.is_recording
        
        # Publish trigger
        msg = Bool()
        msg.data = self.is_recording
        self.trigger_pub.publish(msg)
        
        status = "STARTED" if self.is_recording else "STOPPED"
        self.get_logger().info(f"Button Pressed: Voice Recording {status}")

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
