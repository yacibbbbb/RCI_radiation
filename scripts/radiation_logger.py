#!/usr/bin/env python3
import rclpy, csv, os, math
from rclpy.node import Node
from std_msgs.msg import Float64, Bool
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry

# 누적 방사선량을 기록하기 위한 노드
# waypoint 노드가 포함된 런치 파일에서 함께 run 하고, waypoint에서 goal에 도달하면 함께 종료.
# /data/cumulation/ 위치에 "런치 파일에서 등록한 이름"대로 csv파일 저장
# csv 파일에는 측정 순서대로, 시간 + 위치 + Intensity 저장

# csv 파일을 통해서 python script로 적분 + plot 수행

class RadiationLogger(Node):
    def __init__(self):
        super().__init__('radiation_logger')

        # --- Parameters ---
        self.declare_parameter('intensity_topic', '/radiation_sensor/detected_intensity')
        self.declare_parameter('pose_topic', '/odom')
        self.declare_parameter('use_odom', True)
        self.declare_parameter('sample_dt', 0.2)     # 0.1~0.5 s 권장
        self.declare_parameter('csv_name', 'logs/run.csv')
        self.declare_parameter('listen_mission_done', True)  # waypoint에서 /mission_done 주면 함께 종료

        self.intensity_topic = self.get_parameter('intensity_topic').value
        self.pose_topic = self.get_parameter('pose_topic').value
        self.use_odom = bool(self.get_parameter('use_odom').value)
        self.sample_dt = float(self.get_parameter('sample_dt').value)
        self.csv_name = self.get_parameter('csv_name').value
        self.listen_done = bool(self.get_parameter('listen_mission_done').value)

        # --- State ---
        self.latest_I = 0.0
        self.pose = (float('nan'), float('nan'), float('nan'))  # x,y,yaw
        # self.prev_clock = None
        self.running = True

        # --- Subscriptions ---
        self.create_subscription(Float64, self.intensity_topic, self.cb_intensity, 10)
        if self.use_odom:
            self.create_subscription(Odometry, '/odom', self.cb_odom, 10)
        else:
            self.create_subscription(PoseWithCovarianceStamped, self.pose_topic, self.cb_amcl, 10)
        
        if self.listen_done:
            self.create_subscription(Bool, '/mission_done', self.cb_done, 10)

        # --- Timer ---
        self.timer = self.create_timer(self.sample_dt, self.on_timer)

        # --- CSV ---
        self._init_csv()

        self.get_logger().info(
            f'[radiation_logger] sample_dt={self.sample_dt}s, csv="{self.csv_name}", '
            f'intensity="{self.intensity_topic}", pose="{"/odom" if self.use_odom else self.pose_topic}"'
        )

    # ---------- Callbacks ----------
    def cb_intensity(self, msg: Float64):
        self.latest_I = max(0.0, float(msg.data))  # 음수 방지

    def cb_amcl(self, msg: PoseWithCovarianceStamped):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = self._yaw(q.x, q.y, q.z, q.w)
        self.pose = (p.x, p.y, yaw)

    def cb_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = self._yaw(q.x, q.y, q.z, q.w)
        self.pose = (p.x, p.y, yaw)
    
    def cb_done(self, msg: Bool):
        if msg.data:
            self.get_logger().info('Received /mission_done=True → stop logging.')
            self._finish_and_shutdown()

    def on_timer(self):
        if not self.running:
            return
        
        now = self.get_clock().now()
        t_sec = now.nanoseconds * 1e-9
        x, y, yaw = self.pose

        # NaN이면 기록 건너뜀
        if any(math.isnan(v) for v in (x, y, yaw)):
            return
        
        # 기록
        row = [f'{t_sec:.3f}', f'{self.latest_I:.6f}', f'{x:.4f}', f'{y:.4f}', f'{yaw:.4f}']
        self.writer.writerow(row)

        # 안전을 위한 주기적 flush
        if not hasattr(self, '_row_count'):
            self._row_count = 0
        self._row_count += 1
        if self._row_count % 25 == 0:
            self.csv_file.flush()

    # ---------- Utils ----------
    def _yaw(self, x, y, z, w):
        s = 2.0*(w*z + x*y)
        c = 1.0 - 2.0*(y*y + z*z)
        return math.atan2(s, c)

    def _init_csv(self):
        d = os.path.dirname(self.csv_name)
        if d and not os.path.exists(d):
            os.makedirs(d, exist_ok=True)
        self.csv_file = open(self.csv_name, 'w', newline='')
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow(['t_sec','intensity','x','y','yaw'])

    def _finish_and_shutdown(self):
        if not self.running:
            return
        self.running = False
        try:
            self.csv_file.flush()
            self.csv_file.close()
        except Exception:
            pass
        self.get_logger().info(f'CSV saved: {os.path.abspath(self.csv_name)}')
        # logger도 함께 종료
        rclpy.shutdown()

def main():
    rclpy.init()
    node = RadiationLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._finish_and_shutdown()

if __name__ == '__main__':
    main()