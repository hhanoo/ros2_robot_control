import rclpy
from rclpy.node import Node

# Messages
from sensor_msgs.msg import Image, CameraInfo

# RealSense sensor class
from realsense_pkg.realsense_class import RSSensor


class RealSenseNode(Node):

    def __init__(self):
        super().__init__('realsense_node')
        self.get_logger().info("Starting RealSense Node...")

        # -------- Parameters --------
        self.declare_parameter('update_rate', 0.02)  # 0.02 seconds (50Hz)

        self.update_rate = self.get_parameter('update_rate').get_parameter_value().double_value

        # -------- Connect to sensor --------
        self.sensor = RSSensor(sensor_index=0)
        if not self.sensor.init():
            self.get_logger().error("Failed to initialize sensor.")
            raise RuntimeError("Sensor initialization failed")
        self.get_logger().info(f"Connected to sensor at {self.sensor.serial_number}")

        # -------- Publishers --------

        # -------- ROS Timer (state update & publishing) --------
        self.create_timer(self.update_rate, self.timer_update)

        self.get_logger().info("RealSense Node Ready.")

    # ============================ Timer callback: Update & Publish ============================

    def timer_update(self):
        pass

    # ============================ Shutdown ============================

    def destroy_node(self):
        try:
            if hasattr(self, 'sensor'):
                self.sensor.stop()
        except Exception as e:
            self.get_logger().warn(f"Error on stop: {e}")
        super().destroy_node()


def main():
    rclpy.init()
    node = RealSenseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node interrupted, shutting down...")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
