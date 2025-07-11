#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from dvl_msgs.msg import DVLDR
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from rclpy.qos import qos_profile_sensor_data

from builtin_interfaces.msg import Time

class PositionToPathNode(Node):
    def __init__(self):
        super().__init__('position_to_path_node')
        self.declare_parameter('frame_id', 'odom')
        self.subscription = self.create_subscription(
            DVLDR,
            '/dvl/position',
            self.listener_callback,
            qos_profile_sensor_data)
        self.publisher = self.create_publisher(Path, '/dvl/path', 10)
        self.path = Path()

    def listener_callback(self, msg):
        pose = PoseStamped()

        # Convert float timestamp to Time message
        secs = int(msg.time)
        nsecs = int((msg.time - secs) * 1e9)
        pose.header.stamp = Time(sec=secs, nanosec=nsecs)

        pose.header.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
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