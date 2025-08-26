from mocap_optitrack_interfaces.msg import RigidBodyArray
from geometry_msgs.msg import Pose
import rclpy
from rclpy.node import Node

class ForwardPublisher(Node):
    def __init__(self):
        super().__init__('forward_publisher')
        self.subscriber = self.create_subscription(RigidBodyArray, '/mocap_rigid_bodies', self.listener_callback, 10)        
        self.declare_parameter("robot_namespaces", ["robot_2", "robot_3", "robot_4", "robot_5","robot_6"])
        self.robot_namespaces = (
            self.get_parameter("robot_namespaces")
            .get_parameter_value()
            .string_array_value
        )        
        self.publishers_dict = {}
        for ns in self.robot_namespaces:
            # cmd_vel_topic = f"/{ns}/cmd_vel"
            pose_topic = f"/{ns}/pose"            # Create a publisher for cmd_vel
            self.publishers_dict[ns] = self.create_publisher(Pose, pose_topic, 10)    
    def listener_callback(self, msg):
        data = msg.rigid_bodies
        for i, _ in enumerate(data):
            ns = self.robot_namespaces[i]
            pose = Pose()
            pose.position.x = data[i].pose_stamped.pose.position.x
            pose.position.y = data[i].pose_stamped.pose.position.y
            pose.position.z = data[i].pose_stamped.pose.position.z
            pose.orientation.x = data[i].pose_stamped.pose.orientation.x
            pose.orientation.y = data[i].pose_stamped.pose.orientation.y
            pose.orientation.z = data[i].pose_stamped.pose.orientation.z
            pose.orientation.w = data[i].pose_stamped.pose.orientation.w
            self.publishers_dict[ns].publish(pose)
            
def main(args=None):
    rclpy.init(args=args)
    forwarder_node = ForwardPublisher()
    rclpy.spin(forwarder_node)
    forwarder_node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
