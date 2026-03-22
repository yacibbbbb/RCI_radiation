#!/usr/bin/env python3
import os
import time
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from std_msgs.msg import Float64, ColorRGBA
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
from pathlib import Path

WORLD_NAME = 'single_source'

SAVE_DIR = Path(__file__).resolve().parents[4]

class Sweeper(Node):
    """
    이벤트 드리븐 스위퍼 (셀 중심 샘플링):
      - 현재 셀(중앙)의 포즈를 /virtual_sensor/pose 로 발행
      - /virtual_sensor/detected_intensity 수신 즉시 다음 셀로 진행
      - 행 단위로 마커를 업데이트(원하면 더 자주도 가능)
      - 완료 시 CSV 저장
    """

    def __init__(self):
        super().__init__('sweeper_node')

        # ------------------------
        # 런치 파라미터
        # ------------------------
        self.declare_parameter('map_resolution', 0.25)
        self.declare_parameter('map_width', 19.0)
        self.declare_parameter('map_height', 20.0)
        self.declare_parameter('map_origin_x', -9.5)
        self.declare_parameter('map_origin_y', -10.0)

        self.declare_parameter('fixed_z', 0.5)
        self.declare_parameter('map_frame', 'map')

        self.declare_parameter('max_intensity', 100.0)

        # RViz 마커
        self.declare_parameter('publish_markers', True)
        self.declare_parameter('marker_stride', 1)       # 몇 칸마다 marking 할 것인가
        self.declare_parameter('marker_ns', 'rad_sweep')

        # 토픽
        self.declare_parameter('virtual_pose_topic', '/virtual_sensor/pose')
        self.declare_parameter('virtual_intensity_topic', '/virtual_sensor/detected_intensity')
        self.declare_parameter('marker_topic', '/sweep/markers')

        # 저장
        self.declare_parameter('world_name', 'default_world')
        self.declare_parameter(
            'base_dir',
            os.path.join(SAVE_DIR, 'src/radiation_simulation/data')
        )
        self.declare_parameter('save_csv', True)

        # 타임아웃/워치독
        self.declare_parameter('timeout_sec', 0.01)      
        self.declare_parameter('watchdog_hz', 20.0)      # 20 Hz 감시

        # ------------------------
        # 파라미터 적용/좌표계
        # ------------------------
        self.res = float(self.get_parameter('map_resolution').value)
        width   = float(self.get_parameter('map_width').value)
        height  = float(self.get_parameter('map_height').value)
        self.xmin = float(self.get_parameter('map_origin_x').value)
        self.ymin = float(self.get_parameter('map_origin_y').value)
        self.xmax = self.xmin + width
        self.ymax = self.ymin + height

        self.fixed_z   = float(self.get_parameter('fixed_z').value)
        self.map_frame = str(self.get_parameter('map_frame').value)

        self.max_intensity = float(self.get_parameter('max_intensity').value)

        self.publish_markers = bool(self.get_parameter('publish_markers').value)
        self.stride          = int(self.get_parameter('marker_stride').value)  # 고정 1
        self.marker_ns       = str(self.get_parameter('marker_ns').value)

        self.pose_topic      = str(self.get_parameter('virtual_pose_topic').value)
        self.intensity_topic = str(self.get_parameter('virtual_intensity_topic').value)
        self.marker_topic    = str(self.get_parameter('marker_topic').value)

        self.world_name = str(self.get_parameter('world_name').value)
        self.base_dir   = str(self.get_parameter('base_dir').value)
        self.save_csv   = bool(self.get_parameter('save_csv').value)

        self.timeout_sec = float(self.get_parameter('timeout_sec').value)
        watchdog_hz      = float(self.get_parameter('watchdog_hz').value)
        watchdog_period  = max(0.001, 1.0 / watchdog_hz)

        # ------------------------
        # 그리드(셀-센터 샘플링)
        # ------------------------
        # 셀 개수는 경계선 제외: 셀 중심을 (i+0.5)*res 로 배치
        self.nx = int(np.floor((self.xmax - self.xmin) / self.res))
        self.ny = int(np.floor((self.ymax - self.ymin) / self.res))
        self.cols = self.nx
        self.rows = self.ny

        self.get_logger().info(
            f'Grid(centered): {self.nx} x {self.ny} @ {self.res} m '
            f'origin=({self.xmin:.2f},{self.ymin:.2f})'
        )

        # 강도 배열
        self.I = np.zeros((self.rows, self.cols), dtype=np.float64)

        # 진행 인덱스
        self.i = 0
        self.j = 0

        # 상태
        self.awaiting = False
        self.last_request_time = None

        # ------------------------
        # QoS & 통신
        # ------------------------
        qos_sensor = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        # 마커는 RViz 호환을 위해 RELIABLE + TRANSIENT_LOCAL
        qos_markers = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )

        self.pose_pub = self.create_publisher(PoseStamped, self.pose_topic, qos_sensor)
        self.intensity_sub = self.create_subscription(
            Float64, self.intensity_topic, self._on_intensity, qos_sensor
        )
        self.marker_pub = self.create_publisher(MarkerArray, self.marker_topic, qos_markers)

        # 첫 셀 요청
        self._request_current_cell()

        # 타임아웃 감시
        self.watchdog = self.create_timer(watchdog_period, self._watchdog_tick)

        self.get_logger().info(
            f'Sweeper ready. pose → {self.pose_topic}, intensity ← {self.intensity_topic}, '
            f'markers → {self.marker_topic}'
        )

    # -------------------------------------------------
    # 셀 중심 좌표 (i: x-index, j: y-index)
    # -------------------------------------------------
    def _grid_xy(self, i: int, j: int):
        # 셀 중심: origin + (i+0.5)*res
        x = self.xmin + (i + 0.5) * self.res
        y = self.ymin + (j + 0.5) * self.res
        return float(x), float(y)

    # -------------------------------------------------
    # 현재 셀 요청(포즈 발행)
    # -------------------------------------------------
    def _request_current_cell(self):
        if self.j >= self.ny:
            if self.publish_markers:
                self._publish_gradient_markers()
            self._maybe_save_csv()
            self.get_logger().info('Sweep complete.')
            rclpy.shutdown()
            return

        x, y = self._grid_xy(self.i, self.j)
        self._publish_pose(x, y, self.fixed_z)
        self.awaiting = True
        self.last_request_time = time.time()

    # -------------------------------------------------
    # 포즈 퍼블리시 (map_frame 고정)
    # -------------------------------------------------
    def _publish_pose(self, x: float, y: float, z: float):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        msg.pose.orientation.w = 1.0
        self.pose_pub.publish(msg)

    # -------------------------------------------------
    # 인텐시티 수신 → 즉시 다음 셀
    # -------------------------------------------------
    def _on_intensity(self, msg: Float64):
        if not self.awaiting:
            return
        self.I[self.j, self.i] = float(msg.data)
        self.awaiting = False
        self._advance()
        self._request_current_cell()

    # -------------------------------------------------
    # 타임아웃 감시
    # -------------------------------------------------
    def _watchdog_tick(self):
        if self.awaiting and (time.time() - self.last_request_time) > self.timeout_sec:
            self.get_logger().warn(f'Timeout @ cell (i={self.i}, j={self.j}), fill 0.0')
            self.I[self.j, self.i] = 0.0
            self.awaiting = False
            self._advance()
            self._request_current_cell()

    # -------------------------------------------------
    # 셀 인덱스 전진 (행 완료 시 마커 갱신)
    # -------------------------------------------------
    def _advance(self):
        self.i += 1
        # 매 측정마다 마커 갱신할 시
        # self._publish_gradient_markers()
        if self.i >= self.nx:
            self.i = 0
            self.j += 1
            if self.publish_markers:
                self._publish_gradient_markers()

        # 진행률 로그(5% 단위)
        total = self.nx * self.ny
        done = min(self.j * self.nx + self.i, total)
        if total > 0 and (done % max(1, total // 20) == 0):
            self.get_logger().info(f'Progress: {done}/{total} cells')

    # -------------------------------------------------
    # Gradient ARROW + SPHERE 마커 발행
    # -------------------------------------------------
    def _publish_gradient_markers(self):
        if self.rows == 0 or self.cols == 0:
            return

        def _rainbow_color(norm: float):
            norm = float(np.clip(norm, 0.0, 1.0))

            # blue -> cyan -> green -> yellow -> orange -> red
            anchors = [
                (0.00, (0.0, 0.0, 1.0)),   # blue
                (0.25, (0.0, 1.0, 1.0)),   # cyan
                (0.50, (0.0, 1.0, 0.0)),   # green
                (0.75, (1.0, 1.0, 0.0)),   # yellow
                (0.90, (1.0, 0.5, 0.0)),   # orange
                (1.00, (1.0, 0.0, 0.0)),   # red
            ]

            for k in range(len(anchors) - 1):
                t0, c0 = anchors[k]
                t1, c1 = anchors[k + 1]
                if t0 <= norm <= t1:
                    u = (norm - t0) / (t1 - t0 + 1e-9)
                    r = c0[0] + u * (c1[0] - c0[0])
                    g = c0[1] + u * (c1[1] - c0[1])
                    b = c0[2] + u * (c1[2] - c0[2])
                    return r, g, b

            return anchors[-1][1]

        marray = MarkerArray()
        now = self.get_clock().now().to_msg()
        mid = 0
        step = max(1, self.stride)
        sphere_scale = self.res * 0.4    # 구체 반지름

        for j in range(0, self.rows, step):
            for i in range(0, self.cols, step):
                intensity = float(self.I[j, i])
                x, y = self._grid_xy(i, j)

                norm = np.clip(intensity / (self.max_intensity + 1e-9), 0.0, 1.0)
                r, g, b = _rainbow_color(norm)

                mk = Marker()
                mk.header.frame_id = self.map_frame
                mk.header.stamp = now
                mk.ns = self.marker_ns
                mk.id = mid; mid += 1
                mk.type = Marker.SPHERE
                mk.action = Marker.ADD
                mk.pose.position.x = x
                mk.pose.position.y = y
                mk.pose.position.z = self.fixed_z
                mk.pose.orientation.w = 1.0
                mk.scale.x = sphere_scale
                mk.scale.y = sphere_scale
                mk.scale.z = sphere_scale

                mk.color = ColorRGBA(
                    r=float(r),
                    g=float(g),
                    b=float(b),
                    a=0.8
                )

                mk.lifetime.sec = 0
                mk.lifetime.nanosec = 0
                marray.markers.append(mk)

        self.marker_pub.publish(marray)


    # -------------------------------------------------
    # CSV 저장
    # -------------------------------------------------
    def _maybe_save_csv(self):
        if not self.save_csv:
            self.get_logger().info('CSV saving disabled.')
            return
        out_dir = os.path.join(self.base_dir, self.world_name)
        os.makedirs(out_dir, exist_ok=True)
        out_csv = os.path.join(out_dir, 'entire_intensity.csv')
        np.savetxt(out_csv, self.I, delimiter=',', fmt='%.8f')
        self.get_logger().info(f'Saved intensity grid to: {out_csv}')


def main():
    rclpy.init()
    node = Sweeper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node._maybe_save_csv()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
