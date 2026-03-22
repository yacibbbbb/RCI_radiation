#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray
import xml.etree.ElementTree as ET
import os, math
from typing import List, Optional, Tuple

def _parse_pose(text: Optional[str]) -> Tuple[float,float,float,float,float,float]:
    if not text:
        return (0.0,0.0,0.0, 0.0,0.0,0.0)
    vals = [float(p) for p in text.strip().split()]
    while len(vals) < 6:
        vals.append(0.0)
    return tuple(vals[:6])

def _rpy_to_quat(r, p, y):
    cy, sy = math.cos(y*0.5), math.sin(y*0.5)
    cp, sp = math.cos(p*0.5), math.sin(p*0.5)
    cr, sr = math.cos(r*0.5), math.sin(r*0.5)
    w = cr*cp*cy + sr*sp*sy
    x = sr*cp*cy - cr*sp*sy
    yq = cr*sp*cy + sr*cp*sy
    z = cr*cp*sy - sr*sp*cy
    return (x, yq, z, w)

def _resolve_mesh_uri(uri: str, world_path: str, model_roots: List[str], package_name: str) -> Optional[str]:
    """
    RViz Marker가 이해하는 리소스로 변환:
      - package:// 그대로 유지
      - model://X/... -> package://<package_name>/models/X/...
      - file://... 또는 상대경로 -> file://<절대경로>
    """
    if not uri:
        return None
    uri = uri.strip()
    if uri.startswith('package://'):
        return uri

    if uri.startswith('model://'):
        remain = uri[len('model://'):]  # "X/meshes/.."
        parts = remain.split('/', 1)
        model_name = parts[0]
        tail = parts[1] if len(parts) > 1 else ''
        # package://<pkg>/models/<model_name>/<tail>
        if package_name:
            return f'package://{package_name}/worlds/{model_name}/{tail}'
        # file:// 로도 시도
        for root in model_roots:
            fpath = os.path.join(root, model_name, tail)
            if os.path.exists(fpath):
                return 'file://' + os.path.abspath(fpath)
        return None

    # file:// 또는 상대경로
    if uri.startswith('file://'):
        path = uri[len('file://'):]
        if os.path.isabs(path):
            return uri
        base = os.path.dirname(world_path)
        fpath = os.path.join(base, path)
        return 'file://' + os.path.abspath(fpath)

    # 상대경로
    base = os.path.dirname(world_path)
    fpath = os.path.join(base, uri)
    if os.path.exists(fpath):
        return 'file://' + os.path.abspath(fpath)
    return None

class ObstacleMarkerPublisher(Node):
    """
    SDF world를 파싱하여 BOX/MESH를 MarkerArray로 발행.
    - QoS: 기본 VOLATILE(KeepLast=10, Reliable), + 주기 재발행으로 RViz 기본 설정에서도 항상 보이게 함
    - mesh/box를 못 찾으면 SPHERE 폴백 없이 건너뛰고 경고
    """
    def __init__(self):
        super().__init__('obstacle_marker_publisher')

        # ---------------- Parameters ----------------
        self.declare_parameter('world_path', '')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('marker_topic', '/obstacle_markers')
        self.declare_parameter('ns', 'obstacles')
        self.declare_parameter('color_rgba', [0.3,0.3,0.3,0.8])
        # RViz 기본 QoS(Volatile)과 바로 호환시키기 위해 publisher도 Volatile + 주기 재발행
        self.declare_parameter('publish_rate', 1.0)  # Hz
        # mesh uri 해석용
        self.declare_parameter('package_name', 'radiation_simulation')
        self.declare_parameter('model_roots', [])  # 예: [<pkg_share>/models]
        # 장애물로 취급할 모델 이름 패턴
        self.declare_parameter('name_filters', ['obstacle','radiation_obstacle','wall','shield','block'])

        self.frame_id = self.get_parameter('map_frame').get_parameter_value().string_value
        self.world_path = self.get_parameter('world_path').get_parameter_value().string_value
        self.ns = self.get_parameter('ns').get_parameter_value().string_value
        rgba = list(self.get_parameter('color_rgba').get_parameter_value().double_array_value)
        self.color = tuple((rgba + [0,0,0,0])[:4])
        self.rate = float(self.get_parameter('publish_rate').value)
        self.package_name = self.get_parameter('package_name').get_parameter_value().string_value
        self.model_roots = list(self.get_parameter('model_roots').get_parameter_value().string_array_value)
        self.name_filters = list(self.get_parameter('name_filters').get_parameter_value().string_array_value)

        # ---------------- QoS (VOLATILE + KeepLast=10) ----------------
        volatile_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        # durability 기본은 VOLATILE (설정 안 함)
        self.marker_topic = self.get_parameter('marker_topic').get_parameter_value().string_value
        self.pub_markers = self.create_publisher(MarkerArray, self.marker_topic, volatile_qos)

        ok, markers = self._build_markers_from_world(self.world_path)
        if not ok:
            self.get_logger().error(f"SDF world 파싱 실패: {self.world_path}")
            markers = []

        # 최초 1회 발행 + 이후 주기 재발행
        self._markers_cache = markers
        self._publish_once(delete_all=True)
        if self.rate > 0.0:
            self._timer = self.create_timer(1.0/self.rate, self._publish_once)
        self.get_logger().info(f"장애물 마커 준비 완료: topic={self.marker_topic}, count={len(markers)} (QoS=VOLATILE, republish={self.rate}Hz)")

    # ---------------------------------------------------------------
    # World → MarkerArray
    # ---------------------------------------------------------------
    def _build_markers_from_world(self, path: str) -> Tuple[bool, List[Marker]]:
        if not path or not os.path.exists(path):
            return (False, [])
        try:
            root = ET.parse(path).getroot()
            world = root.find('./world') if root.tag.endswith('sdf') else (root if root.tag.endswith('world') else None)
            if world is None:
                return (False, [])

            mks: List[Marker] = []
            mid = 0
            now = self.get_clock().now().to_msg()

            def add_marker(mk: Marker):
                nonlocal mid
                mk.header.frame_id = self.frame_id
                mk.header.stamp = now
                mk.ns = self.ns
                mk.id = mid; mid += 1
                mk.action = Marker.ADD
                mk.color.r, mk.color.g, mk.color.b, mk.color.a = self.color
                mks.append(mk)

            # 모델 순회
            for m in world.findall('./model'):
                model_name = m.attrib.get('name', '')
                if self.name_filters and not any(p in model_name for p in self.name_filters):
                    continue

                mx,my,mz,mr,mp,myaw = _parse_pose(m.findtext('pose') or '')

                # 링크들
                for link in m.findall('./link'):
                    lx,ly,lz,lr,lp,lyaw = _parse_pose(link.findtext('pose') or '')
                    # (1) collision 여러 개 모두 순회
                    for col in link.findall('./collision'):
                        cx,cy,cz,cr,cp,cyaw = _parse_pose(col.findtext('pose') or '')
                        g = col.find('./geometry')
                        if g is None:
                            continue
                        # 박스
                        box = g.find('./box')
                        if box is not None:
                            size_text = (box.findtext('size') or '').strip()
                            if size_text:
                                try:
                                    sx,sy,sz = [abs(float(v)) for v in size_text.split()[:3]]
                                    mk = Marker()
                                    mk.type = Marker.CUBE
                                    # 합성 pose (단순 합: sdf 기본은 부모 기준 상대)
                                    px = mx + lx + cx; py = my + ly + cy; pz = mz + lz + cz
                                    pr = mr + lr + cr; pp = mp + lp + cp; pyw = myaw + lyaw + cyaw
                                    qx,qy,qz,qw = _rpy_to_quat(pr,pp,pyw)
                                    mk.pose.position.x = px; mk.pose.position.y = py; mk.pose.position.z = pz
                                    mk.pose.orientation.x = qx; mk.pose.orientation.y = qy; mk.pose.orientation.z = qz; mk.pose.orientation.w = qw
                                    mk.scale.x = max(1e-4, sx); mk.scale.y = max(1e-4, sy); mk.scale.z = max(1e-4, sz)
                                    add_marker(mk)

                                except Exception as e:
                                    self.get_logger().warn(f"[{model_name}] box size 파싱 실패: {e}")

                        # 메시
                        mesh = g.find('./mesh')
                        if mesh is not None:
                            uri = (mesh.findtext('uri') or '').strip()
                            scale_txt = (mesh.findtext('scale') or '').strip()
                            sx,sy,sz = 1.0, 1.0, 1.0
                            if scale_txt:
                                try:
                                    vals = [float(v) for v in scale_txt.split()]
                                    if len(vals) == 1:
                                        sx = sy = sz = max(1e-6, vals[0])
                                    else:
                                        sx,sy,sz = [max(1e-6, abs(v)) for v in vals[:3]]
                                except: pass

                            res = _resolve_mesh_uri(uri, self.world_path, self.model_roots, self.package_name)
                            if not res:
                                self.get_logger().warn(f"[{model_name}] mesh uri 해석 실패: '{uri}'")
                            else:
                                mk = Marker()
                                mk.type = Marker.MESH_RESOURCE
                                px = mx + lx + cx; py = my + ly + cy; pz = mz + lz + cz
                                pr = mr + lr + cr; pp = mp + lp + cp; pyw = myaw + lyaw + cyaw
                                qx,qy,qz,qw = _rpy_to_quat(pr,pp,pyw)
                                mk.pose.position.x = px; mk.pose.position.y = py; mk.pose.position.z = pz
                                mk.pose.orientation.x = qx; mk.pose.orientation.y = qy; mk.pose.orientation.z = qz; mk.pose.orientation.w = qw
                                mk.scale.x = sx; mk.scale.y = sy; mk.scale.z = sz
                                mk.mesh_resource = res
                                mk.mesh_use_embedded_materials = True
                                add_marker(mk)

                    # (2) visual도 순회 (collision에 없을 때 보완)
                    for vis in link.findall('./visual'):
                        vx,vy,vz,vr,vp,vyaw = _parse_pose(vis.findtext('pose') or '')
                        g = vis.find('./geometry')
                        if g is None: continue

                        # 박스
                        box = g.find('./box')
                        if box is not None:
                            size_text = (box.findtext('size') or '').strip()
                            if size_text:
                                try:
                                    sx,sy,sz = [abs(float(v)) for v in size_text.split()[:3]]
                                    mk = Marker()
                                    mk.type = Marker.CUBE
                                    px = mx + lx + vx; py = my + ly + vy; pz = mz + lz + vz
                                    pr = mr + lr + vr; pp = mp + lp + vp; pyw = myaw + lyaw + vyaw
                                    qx,qy,qz,qw = _rpy_to_quat(pr,pp,pyw)
                                    mk.pose.position.x = px; mk.pose.position.y = py; mk.pose.position.z = pz
                                    mk.pose.orientation.x = qx; mk.pose.orientation.y = qy; mk.pose.orientation.z = qz; mk.pose.orientation.w = qw
                                    mk.scale.x = max(1e-4, sx); mk.scale.y = max(1e-4, sy); mk.scale.z = max(1e-4, sz)
                                    add_marker(mk)
                                except Exception as e:
                                    self.get_logger().warn(f"[{model_name}] visual box size 파싱 실패: {e}")

                        # 메시
                        mesh = g.find('./mesh')
                        if mesh is not None:
                            uri = (mesh.findtext('uri') or '').strip()
                            scale_txt = (mesh.findtext('scale') or '').strip()
                            sx,sy,sz = 1.0, 1.0, 1.0
                            if scale_txt:
                                try:
                                    vals = [float(v) for v in scale_txt.split()]
                                    if len(vals) == 1:
                                        sx = sy = sz = max(1e-6, vals[0])
                                    else:
                                        sx,sy,sz = [max(1e-6, abs(v)) for v in vals[:3]]
                                except: pass
                            res = _resolve_mesh_uri(uri, self.world_path, self.model_roots, self.package_name)
                            if not res:
                                self.get_logger().warn(f"[{model_name}] visual mesh uri 해석 실패: '{uri}'")
                            else:
                                mk = Marker()
                                mk.type = Marker.MESH_RESOURCE
                                px = mx + lx + vx; py = my + ly + vy; pz = mz + lz + vz
                                pr = mr + lr + vr; pp = mp + lp + vp; pyw = myaw + lyaw + vyaw
                                qx,qy,qz,qw = _rpy_to_quat(pr,pp,pyw)
                                mk.pose.position.x = px; mk.pose.position.y = py; mk.pose.position.z = pz
                                mk.pose.orientation.x = qx; mk.pose.orientation.y = qy; mk.pose.orientation.z = qz; mk.pose.orientation.w = qw
                                mk.scale.x = sx; mk.scale.y = sy; mk.scale.z = sz
                                mk.mesh_resource = res
                                mk.mesh_use_embedded_materials = True
                                add_marker(mk)

            return (True, mks)

        except Exception as e:
            self.get_logger().error(f'world 파싱 에러: {e}')
            return (False, [])

    # ---------------------------------------------------------------
    def _publish_once(self, delete_all: bool=False):
        ma = MarkerArray()
        now = self.get_clock().now().to_msg()
        if delete_all:
            mk_del = Marker()
            mk_del.header.frame_id = self.frame_id
            mk_del.header.stamp = now
            mk_del.action = Marker.DELETEALL
            ma.markers.append(mk_del)
        for mk in self._markers_cache:
            mk.header.stamp = now
            ma.markers.append(mk)
        self.pub_markers.publish(ma)

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleMarkerPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
