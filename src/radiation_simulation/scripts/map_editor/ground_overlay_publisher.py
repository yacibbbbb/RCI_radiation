#!/usr/bin/env python3
import rclpy, math, os
from rclpy.node import Node
from visualization_msgs.msg import Marker

from pathlib import Path

# Rviz에서 바닥에 Radiation Map을 시각화할 수 있게
# Marker 퍼블리시하는 노드

class GroundOverlay(Node):
    def __init__(self):
        super().__init__('ground_overlay')
        self.pub = self.create_publisher(Marker, 'png_ground', 1)
        self.timer = self.create_timer(0.5, self.tick)

        radiation_sim_dir = Path(__file__).resolve().parents[1]
        uri_tmp = os.path.join(radiation_sim_dir, 'models/overlays', 'textured_plane.dae')

        # 필요한 값만 수정해서 쓰면 됨
        self.frame_id = 'map'
        self.mesh_uri = 'file://' + uri_tmp
        width_m, height_m = 19.0, 20.0   # PNG 실제 보여줄 크기(m)
        origin_x, origin_y = -9.5, -10.0 # 좌하단 원점(map 좌표)
        self.cx = origin_x + width_m*0.5
        self.cy = origin_y + height_m*0.5
        self.scale_x, self.scale_y = width_m, height_m
        self.z = -0.01
        self.yaw = 0.0

    def tick(self):
        m = Marker()
        m.header.frame_id = self.frame_id
        m.type = Marker.MESH_RESOURCE
        m.mesh_resource = self.mesh_uri
        m.mesh_use_embedded_materials = True
        m.pose.position.x, m.pose.position.y, m.pose.position.z = self.cx, self.cy, self.z
        m.pose.orientation.w = 1.0
        m.scale.x, m.scale.y, m.scale.z = self.scale_x, self.scale_y, 1.0
        m.color.r = m.color.g = m.color.b = m.color.a = 1.0
        self.pub.publish(m)

def main():
    rclpy.init()
    rclpy.spin(GroundOverlay())
    rclpy.shutdown()

if __name__ == '__main__': 
    main()
