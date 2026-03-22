#!/usr/bin/env python3

# Trajectory를 Rviz에 시각화 하는 용도

# Rviz에서 Add - By topic - path/odom_path 선택 후 추가

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped

class OdomPath(Node):
    def __init__(self):
        super().__init__('odom_path_node')
        self.path_pub = self.create_publisher(Path, 'odom_path', 10)
        self.sub = self.create_subscription(Odometry, 'odom', self.odom_cb, 10)
        self.path = Path()
        self.path.header.frame_id = 'map'  # 또는 'odom'

    def odom_cb(self, msg: Odometry):
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        self.path.poses.append(pose)
        self.path.header.stamp = msg.header.stamp
        self.path_pub.publish(self.path)

def main():
    rclpy.init()
    node = OdomPath()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
