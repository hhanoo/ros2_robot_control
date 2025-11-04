import rclpy
from rclpy.node import Node

# Services
from std_srvs.srv import Trigger

# Messages
from std_msgs.msg import Float64MultiArray, Int32MultiArray, Int32, Bool
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose

# Low-level TCP controller class
from ur_bridge_py.ur_class import URClass


class URBridgeNode(Node):

    def __init__(self):
        super().__init__('ur_bridge_node')
        self.get_logger().info("Starting UR TCP Bridge Node...")

        # -------- Parameters --------
        self.declare_parameter('ur_ip', '127.0.0.1')
        self.declare_parameter('update_rate', 0.05)  # 0.05 seconds (20Hz)

        ur_ip = self.get_parameter('ur_ip').get_parameter_value().string_value
        self.update_rate = self.get_parameter('update_rate').get_parameter_value().double_value

        # -------- Connect to robot --------
        self.robot = URClass(ip=ur_ip)
        if not self.robot.connect():
            self.get_logger().error("Failed to connect to UR robot.")
            raise RuntimeError("UR connection failed")
        self.get_logger().info(f"Connected to UR robot at {ur_ip}:30003")

        # -------- Publishers --------
        self.act_joint_pub = self.create_publisher(JointState, '/ur/actual_joint', 10)
        self.act_pose_pub = self.create_publisher(Pose, '/ur/actual_pose', 10)
        self.act_T_pub = self.create_publisher(Float64MultiArray, '/ur/actual_T', 10)
        self.digital_input_pub = self.create_publisher(Int32MultiArray, '/ur/digital_input', 10)
        self.robot_state_pub = self.create_publisher(Int32, '/ur/robot_state', 10)
        self.cmd_send_flag_pub = self.create_publisher(Bool, '/ur/cmd_send_flag', 10)

        # -------- Services --------
        self.create_service(Trigger, 'get_robot_state', self.get_robot_state_cb)

        # -------- ROS Subscribers --------
        self.create_subscription(Float64MultiArray, '/ur/desired_joint', self.desired_joint_cb, 10)
        self.create_subscription(Float64MultiArray, '/ur/desired_pose', self.desired_pose_cb, 10)
        self.create_subscription(Float64MultiArray, '/ur/desired_T', self.desired_T_cb, 10)
        self.create_subscription(Int32MultiArray, '/ur/digital_output', self.digital_output_cb, 10)

        # -------- ROS Timer (state update & publishing) --------
        self.create_timer(self.update_rate, self.timer_update)

        self.get_logger().info("UR TCP Bridge Node Ready.")

    # ============================ Services ============================

    def get_robot_state_cb(self, req: Trigger.Request, res: Trigger.Response):
        info = (f"mode={self.robot.print_robot_mode()}, safety={self.robot.safety_mode}, prog={self.robot.prog_state},"
                f"actual joint={self.robot.act_q},"
                f"actual pose={self.robot.act_X},"
                f"actual T={self.robot.act_T.flatten().tolist()},"
                f"digital input={self.robot.digital_input}")
        res.success = True
        res.message = info
        return res

    # ============================ ROS Subscribers → Commands ============================

    def desired_joint_cb(self, msg: Float64MultiArray):
        if len(msg.data) != 6:
            self.get_logger().error("Desired joint must be 6-dimensional")
            return
        self.robot.movej(msg.data)
        self.get_logger().info("movej command sent")

    def desired_pose_cb(self, msg: Float64MultiArray):
        if len(msg.data) != 6:
            self.get_logger().error("Desired pose must be 6-dimensional")
            return
        self.robot.movel_pose(msg.data)
        self.get_logger().info("movel(pose) command sent")

    def desired_T_cb(self, msg: Float64MultiArray):
        if len(msg.data) != 16:
            self.get_logger().error("Desired T must be 16-dimensional")
            return
        self.robot.movel_T(msg.data)
        self.get_logger().info("movel(T) command sent")

    def digital_output_cb(self, msg: Int32MultiArray):
        if len(msg.data) != 2:
            self.get_logger().error("Digital output must be 2-dimensional")
            return
        port, val = int(msg.data[0]), bool(int(msg.data[1]))
        self.robot.controlbox_digital_out(port, val)
        self.get_logger().info(f"Digital output command sent: port={port}, value={val}")

    # ============================ Timer callback: Update & Publish ============================

    def timer_update(self):
        # -------- Publish JointState --------
        js = JointState()
        js.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        js.position = list(self.robot.act_q)
        self.act_joint_pub.publish(js)

        # -------- Publish Pose --------
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = self.robot.act_X[0:3]
        pose.orientation.x, pose.orientation.y, pose.orientation.z = self.robot.act_X[3:6]
        self.act_pose_pub.publish(pose)

        # ---- Publish 4x4 Transformation Matrix ----
        Tmsg = Float64MultiArray()
        Tmsg.data = self.robot.act_T.flatten().tolist()
        self.act_T_pub.publish(Tmsg)

        # -------- Publish Digital Input --------
        digital_input = Int32MultiArray()
        digital_input.data = [int(val) for val in self.robot.digital_input]
        self.digital_input_pub.publish(digital_input)

        # -------- Publish Robot State --------
        robot_state = Int32()
        robot_state.data = self.robot.robot_state
        self.robot_state_pub.publish(robot_state)

        # -------- Publish Command Send Flag --------
        cmd_send_flag = Bool()
        cmd_send_flag.data = bool(self.robot.send_move_flag)
        self.cmd_send_flag_pub.publish(cmd_send_flag)

    # ============================ Shutdown ============================

    def destroy_node(self):
        try:
            if hasattr(self, 'robot') and self.robot.is_connected():
                self.robot.disconnect()
        except Exception as e:
            self.get_logger().warn(f"Error on disconnect: {e}")
        super().destroy_node()


def main():
    rclpy.init()
    node = URBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node interrupted, shutting down...")
    finally:
        node.robot.disconnect()
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
