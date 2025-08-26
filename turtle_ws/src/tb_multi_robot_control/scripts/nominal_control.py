#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
import numpy as np
import rowan
import math



class MultiRobotControlNode(Node):
    def __init__(self):
        super().__init__("multi_robot_control_node")

        # Declare and get the list of robot namespaces
        self.declare_parameter("robot_namespaces", ["robot_2","robot_3","robot_4", "robot_5"])
        self.robot_namespaces = (
            self.get_parameter("robot_namespaces")
            .get_parameter_value()
            .string_array_value
        )

        self.declare_parameter("use_mocap", True)
        self.use_mocap = self.get_parameter("use_mocap").get_parameter_value().bool_value

        self.declare_parameter("control_frequency", 20.0)
        self.control_frequency = (
            self.get_parameter("control_frequency").get_parameter_value().double_value
        )

        self.declare_parameter("controller", "nominal")
        self.controller = (
            self.get_parameter("controller").get_parameter_value().string_value
        )
        assert self.controller in ["nominal", "simple"]

        self.target = np.array([[0.0, 0.0],[2.0, 1.0],[-2.0, -1.0],[3.0, -2.0]])
        # Create publishers and subscribers for each robot
        self.publishers_dict = {}
        self.subscribers_dict = {}
        self.robot_states = {}
        for ns in self.robot_namespaces:
            # cmd_vel_topic = f"/{ns}/cmd_vel"
            cmd_vel_topic = f"/{ns}/cmd_vel_mux/input/teleop"
            # Create a publisher for cmd_vel
            self.publishers_dict[ns] = self.create_publisher(Twist, cmd_vel_topic, 10)
            self.get_logger().info(f"Created publisher: {cmd_vel_topic}")
            self.robot_states[ns] = np.array([0.0, 0.0, 0.0])
            if not self.use_mocap:
                odom_topic = f"/{ns}/odom"
                # Create a subscriber for odometry
                self.subscribers_dict[ns] = self.create_subscription(
                    Odometry, odom_topic, self.odom_callback_factory(ns), 10
                )
                self.get_logger().info(f"Created subscriber: {odom_topic}")
            else:
                pose_topic = f"/{ns}/pose"
                # Create a subscriber for pose
                self.subscribers_dict[ns] = self.create_subscription(
                    Pose, pose_topic, self.pose_callback_factory(ns), 10
                )
                self.get_logger().info(f"Created subscriber: {pose_topic}")
        # Timer to periodically publish messages
        self.timer = self.create_timer(
            1 / self.control_frequency, self.publish_control_commands
        )

    def odom_callback_factory(self, namespace):
        """Create a callback for a specific robot's odometry."""

        def odom_callback(msg):
            yaw = rowan.to_euler(
                [
                    msg.pose.pose.orientation.w,
                    msg.pose.pose.orientation.x,
                    msg.pose.pose.orientation.y,
                    msg.pose.pose.orientation.z,
                ],
                "xyz",
            )[2]
            self.get_logger().info(
                f"Odom from {namespace}: Position x={msg.pose.pose.position.x}, y={msg.pose.pose.position.y}, yaw={yaw}",
                throttle_duration_sec=0.5,
            )
            self.robot_states[namespace] = np.array(
                [msg.pose.pose.position.x, msg.pose.pose.position.y, yaw]
            )

        return odom_callback

    def pose_callback_factory(self, namespace):
        """Create a callback for a specific robot's odometry."""

        def pose_callback(msg):
            yaw = rowan.to_euler(
                [
                    msg.orientation.w,
                    msg.orientation.x,
                    msg.orientation.y,
                    msg.orientation.z,
                ],
                "xyz",
            )[2]
            self.get_logger().info(
                f"Pose from {namespace}: Position x={msg.position.x}, y={msg.position.y}, yaw={yaw}",
                throttle_duration_sec=0.5,
            )
            self.robot_states[namespace] = np.array(
                [msg.position.x, msg.position.y, yaw]
            )

        return pose_callback

    def publish_control_commands(self):
        """Publish control commands to all robots."""

        for i, (ns, publisher) in enumerate(self.publishers_dict.items()):
            cmd_msg = Twist()
            if self.controller == "simple":
                cmd_msg.linear.x = 0.3  # Example control message
                cmd_msg.angular.z = 0.5
            elif self.controller == "nominal":
                cmd_msg.linear.x = 0.45  # TODO: This is current max speed
                max_angular_vel = 1.1
                min_angular_vel = -1.1
                Kw = 1.0
                state = self.robot_states[ns]

            delta_x = self.target[i][0] - state[0]
            delta_y = self.target[i][1] - state[1]
            target_angle = math.atan2(delta_y, delta_x)
            angle_error = target_angle - state[2]
            angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))
            omega = Kw * angle_error
            omega = np.clip(omega, min_angular_vel, max_angular_vel)
            cmd_msg.angular.z = omega

            publisher.publish(cmd_msg)
        # self.get_logger().info(f"Published cmd_vel to {ns}")


def main(args=None):
    rclpy.init(args=args)
    node = MultiRobotControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
