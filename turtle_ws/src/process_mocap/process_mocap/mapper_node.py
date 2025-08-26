#!/usr/bin/env python3
import re
from typing import Dict, List, Optional

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from mocap_optitrack_interfaces.msg import RigidBodyArray


def normalize_motive_name(s: Optional[str]) -> str:
    """
    Normalize Motive names so all of these become 'robot_3':
      'robot_3', 'robot 3', 'robot-3', 'Robot3', 'ROBOT_  3'
    """
    s = (s or "").strip().lower()
    s = s.replace(" ", "").replace("-", "_")
    m = re.match(r"^robot_?(\d+)$", s)
    return f"robot_{m.group(1)}" if m else s


def get_pose_from_rb(rb) -> Pose:
    pose = Pose()
    if hasattr(rb, "pose_stamped"):
        ps = rb.pose_stamped.pose
        pose.position.x = ps.position.x
        pose.position.y = ps.position.y
        pose.position.z = ps.position.z
        pose.orientation = ps.orientation
    elif hasattr(rb, "pose"):
        pose = rb.pose
    else:
        if hasattr(rb, "position"):
            pose.position = rb.position
        if hasattr(rb, "orientation"):
            pose.orientation = rb.orientation
    return pose


class MocapMapper(Node):
    """
    Map OptiTrack rigid bodies to robot namespaces.

    Modes (choose ONE):
      - mode: "name"       → Motive name must be 'robot_2', 'robot_3', ...
                             (name is normalized to tolerate spaces/dashes/case)
      - mode: "first_seen" → assign in order of appearance starting at robot_2

    Params:
      - mode (default "first_seen")
      - robot_namespaces (default ["robot_2","robot_3","robot_4","robot_5"])
      - mocap_topic (default "/mocap_rigid_bodies")
    """
    def __init__(self):
        super().__init__("process_mocap")

        self.declare_parameter("mode", "first_seen")
        self.declare_parameter("robot_namespaces", ["robot_2", "robot_3", "robot_4", "robot_5"])
        self.declare_parameter("mocap_topic", "/mocap_rigid_bodies")

        self.mode = self.get_parameter("mode").get_parameter_value().string_value
        self.robot_namespaces = list(
            self.get_parameter("robot_namespaces").get_parameter_value().string_array_value
        )
        self.mocap_topic = self.get_parameter("mocap_topic").get_parameter_value().string_value

        if self.mode not in ("name", "first_seen"):
            self.get_logger().warn(f"Unsupported mode '{self.mode}', using 'first_seen'")
            self.mode = "first_seen"

        self.ns_to_pub: Dict[str, rclpy.publisher.Publisher] = {}
        self.key_to_ns: Dict[str, str] = {}          # key is normalized name or "(unnamed)"
        self.unassigned_ns: List[str] = list(self.robot_namespaces)

        self.sub = self.create_subscription(RigidBodyArray, self.mocap_topic, self._cb, 10)
        self.hb = self.create_timer(2.0, self._heartbeat)
        self.mapping_announced = False

        self.get_logger().info("=== process_mocap ===")
        self.get_logger().info(f"mode={self.mode}")
        self.get_logger().info(f"robot_namespaces={self.robot_namespaces}")
        self.get_logger().info(f"listening on '{self.mocap_topic}'")

    def _publisher_for(self, ns: str):
        if ns not in self.ns_to_pub:
            topic = f"/{ns}/pose"
            self.ns_to_pub[ns] = self.create_publisher(Pose, topic, 10)
            self.get_logger().info(f"[PUB] {topic}")
        return self.ns_to_pub[ns]

    def _choose_ns_by_name(self, rb_name_raw: Optional[str]) -> Optional[str]:
        name = normalize_motive_name(rb_name_raw)
        return name if name in self.robot_namespaces else None

    def _choose_ns_first_seen(self, rb_name_raw: Optional[str]) -> Optional[str]:
        key = normalize_motive_name(rb_name_raw) if rb_name_raw else "(unnamed)"
        if key in self.key_to_ns:
            return self.key_to_ns[key]
        if self.unassigned_ns:
            ns = self.unassigned_ns.pop(0)
            self.key_to_ns[key] = ns
            self.get_logger().info(f"[MAP] assigned '{key}' → {ns}")
            return ns
        return None

    def _cb(self, msg: RigidBodyArray):
        published = 0
        pairs = []
        for rb in msg.rigid_bodies:
            rb_name_raw = getattr(rb, "name", None)
            ns = (
                self._choose_ns_by_name(rb_name_raw)
                if self.mode == "name"
                else self._choose_ns_first_seen(rb_name_raw)
            )
            if not ns:
                self.get_logger().info(
                    f"[SKIP] no namespace for rigid body '{rb_name_raw or '(none)'}'",
                    throttle_duration_sec=1.0,
                )
                continue

            pose = get_pose_from_rb(rb)
            self._publisher_for(ns).publish(pose)
            published += 1
            pairs.append(f"{normalize_motive_name(rb_name_raw) or '(unnamed)'}→{ns}")

        if published:
            self.get_logger().info(
                f"[FWD] {published} poses | {', '.join(pairs)}",
                throttle_duration_sec=0.5,
            )

        if not self.mapping_announced:
            if self.mode == "name":
                self.get_logger().info(
                    "==== Mapping: name-based (Motive names must be robot_2, robot_3, …) ===="
                )
            else:
                self.get_logger().info(
                    "==== Mapping: first-seen order (starts at robot_2) ===="
                )
            if self.key_to_ns:
                self.get_logger().info(f"Current map: {self.key_to_ns}")
            self.mapping_announced = True

    def _heartbeat(self):
        if self.mode == "first_seen":
            self.get_logger().info(f"[HB] map={self.key_to_ns} | remaining={self.unassigned_ns}")
        else:
            self.get_logger().info("[HB] name-based mode; waiting for rigid bodies named robot_*")


def main(args=None):
    rclpy.init(args=args)
    node = MocapMapper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
