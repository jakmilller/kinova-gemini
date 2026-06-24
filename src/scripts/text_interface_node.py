import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys

class TextInterfaceNode(Node):
    def __init__(self):
        super().__init__('text_interface_node')
        self.publisher_ = self.create_publisher(String, '/user_instructions', 10)
        self.timer = self.create_timer(0.1, self.run_interface)
        self.get_logger().info('Text Interface Node started. Type your instruction and press Enter.')

    def run_interface(self):
        # Using a timer to keep the ROS2 loop spinning while waiting for input
        try:
            # We use a blocking input here. 
            # Note: In a real ROS2 app, you'd want this in a separate thread to not block the executor,
            # but for this simple CLI it's acceptable.
            user_input = input("\n[USER]: ")
            if user_input.lower() in ['exit', 'quit']:
                self.get_logger().info('Exiting...')
                rclpy.shutdown()
                sys.exit(0)
            
            msg = String()
            msg.data = user_input
            self.publisher_.publish(msg)
            self.get_logger().info(f'Published instruction: "{user_input}"')
        except EOFError:
            rclpy.shutdown()
            sys.exit(0)

def main(args=None):
    rclpy.init(args=args)
    node = TextInterfaceNode()
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
