# ~/ros2_ws/src/my_debug_pkg/my_debug_pkg/env_check_node.py
import rclpy
from rclpy.node import Node
import os

class EnvCheckNode(Node):
    def __init__(self):
        super().__init__('env_check_node')
        self.get_logger().info("Checking GStreamer Environment Variables:")
        for var in ['GST_PLUGIN_PATH', 'GST_PLUGIN_SYSTEM_PATH', 'GST_REGISTRY', 'GST_DEBUG', 'LD_LIBRARY_PATH']:
            self.get_logger().info(f"{var}: {os.environ.get(var, 'Not Set')}")
        
        # Ensure the node shuts down after printing
        # This is important because we are not spinning the node
        self.destroy_node() 
        rclpy.try_shutdown() # More robust way to signal shutdown if not spinning

def main(args=None):
    rclpy.init(args=args)
    node = EnvCheckNode()
    # No spin needed as the node handles its own shutdown after printing.
    # If rclpy.try_shutdown() isn't enough, you might need a brief spin or wait,
    # but usually for a simple print-and-exit, it's fine.
    # If issues, uncomment below, but it should not be necessary for this simple case.
    # try:
    #    pass # Node does its work in __init__
    # finally:
    #    if rclpy.ok():
    #        node.destroy_node()
    #        rclpy.shutdown()


if __name__ == '__main__':
    main()
