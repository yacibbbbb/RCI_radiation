#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from std_msgs.msg import Bool
import sys
import csv
from ament_index_python.packages import get_package_share_directory
import os

# numpy 버전이 맞지 않아 오류가 발생할 경우 아래의 절차를 통해 수정
# 1. sudo nano /usr/lib/python3/dist-packages/transforms3d/quaternions.py
# 2. np.float 을 float으로 수정
#       ex) _MAX_FLOAT = np.maximum_sctype(np.float) >> _MAX_FLOAT = np.maximum_sctype(float)
 
class WaypointNavigator(Node):
    def __init__(self):
        super().__init__('waypoint_navigator')

        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # ----- mission_done 퍼블리셔 -----
        # radiation_logger.py에 보낼 goal 도착 여부
        self.mission_pub = self.create_publisher(Bool, '/mission_done', 1)

        # 기본 CSV 경로 설정
        radiation_sim_dir = get_package_share_directory('radiation_simulation')
        waypoint_name = 'test' + '.csv'
        default_csv_path = os.path.join(radiation_sim_dir, 'data/waypoint', waypoint_name)

        # CSV 경로를 런치 인자로부터 받음
        if len(sys.argv) > 1:
            self.csv_path = sys.argv[1]
            self.get_logger().info(f"[✓] 런치 인자로 받은 CSV 경로: {self.csv_path}")
        else:
            self.get_logger().warn("CSV 경로 인자가 없어 기본 경로로 설정합니다.")
            self.csv_path = default_csv_path

        self.waypoints = self.load_waypoints_from_csv(self.csv_path)

        self.index = 0
        self.timer = self.create_timer(3.0, self.send_next_goal)  # 3초 간격
        self.waiting = False

    # csv 파일로부터 waypoint 불러오기
    def load_waypoints_from_csv(self, file_path):
        waypoints = []
        try:
            with open(file_path, newline='') as csvfile:
                reader = csv.DictReader(csvfile)
                for row in reader:
                    x = float(row['x'])
                    y = float(row['y'])
                    yaw = float(row['yaw'])
                    waypoints.append((x, y, yaw))
            self.get_logger().info(f"Waypoint {len(waypoints)}개 불러옴")
        except Exception as e:
            self.get_logger().error(f"CSV 파일 로딩 실패: {e}")
        return waypoints

    # goal 갱신
    def send_next_goal(self):
        if self.waiting:
            return
        if self.index >= len(self.waypoints):
            self.get_logger().info('모든 waypoint 탐색 완료')
            # 모두 완료 → mission_done True, 종료
            self._publish_mission_done_and_shutdown(success=True)
            self.timer.cancel()
            return

        x, y, yaw = self.waypoints[self.index]
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.position.z = 0.0

        import math
        from tf_transformations import quaternion_from_euler
        q = quaternion_from_euler(0, 0, yaw)
        goal.pose.pose.orientation.x = q[0]
        goal.pose.pose.orientation.y = q[1]
        goal.pose.pose.orientation.z = q[2]
        goal.pose.pose.orientation.w = q[3]

        self.get_logger().info(f'Waypoint {self.index+1}/{len(self.waypoints)}: ({x:.2f}, {y:.2f})로 이동 시작')
        self.waiting = True
        self.client.wait_for_server()
        self._send_goal_future = self.client.send_goal_async(goal)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    # goal response 콜백
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal 거절됨')
            self._publish_mission_done_and_shutdown(success=False) # 누적 로그도 종료
            self.waiting = False
            return
        self.get_logger().info('Goal 수락됨, 결과 대기 중...')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    # goal result 콜백
    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Goal 완료')
        self.index += 1
        self.waiting = False

    # mission_done 퍼블리시
    def _publish_mission_done_and_shutdown(self, success: bool):
        try:
            self.mission_pub.publish(Bool(data=True))  # 로거가 True면 종료하도록 설계
            self.get_logger().info(f'/mission_done: True 퍼블리시 (성공={success})')
        except Exception as e:
            self.get_logger().warn(f'/mission_done 퍼블리시 실패: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = WaypointNavigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()