#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from dvl_msgs.msg import DVLDR
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from rclpy.qos import qos_profile_sensor_data

class PositionToPathNode(Node):
    def __init__(self):
        super().__init__('dvl_pose_publisher')
        self.subscription = self.create_subscription(
            DVLDR,
            '/dvl/position',
            self.listener_callback,
            qos_profile_sensor_data)
        self.publisher = self.create_publisher(Path, '/dvl/path', 10)
        self.path = Path()

    def listener_callback(self, msg):
        pose = PoseStamped()

        # --- CORRECTED TIMESTAMP ---
        # Use the timestamp from the incoming message header.
        # This preserves the original timing of the data.
        pose.header.stamp = msg.header.stamp

        pose.header.frame_id = self.get_parameter_or('frame_id', 'odom')
        pose.pose.position.x = msg.position.x
        pose.pose.position.y = msg.position.y
        pose.pose.position.z = msg.position.z

        # Update the path's header with the latest pose's header info
        self.path.header = pose.header
        self.path.poses.append(pose)
        self.publisher.publish(self.path)

def main(args=None):
    rclpy.init(args=args)
    node = PositionToPathNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()