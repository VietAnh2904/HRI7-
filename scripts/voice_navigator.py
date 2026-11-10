#!/usr/bin/env python3
"""Bài 7 – Điều hướng bằng giọng nói (Voice Navigation): /voice/text -> phân tích câu lệnh -> Nav2.

  ros2 run tb4_lab04 voice_navigator.py
  ros2 topic pub --once /voice/text std_msgs/msg/String "{data: 'Đi đến khu vực A'}"   # thử không cần micro

Luồng xử lý một câu lệnh:
  1. voice_command.parse_command(): "Di chuyển từ khu vực A đến phòng thí nghiệm" -> start=A, targets=[I].
     Khu vực không tồn tại -> KHÔNG di chuyển, trả lời "Không có khu vực ... Các khu vực hiện có: ...".
  2. Tra khu vực hiện tại của robot (TF map->base_link trên ảnh nhãn maps/zones_labels.pgm).
     voice_command.plan_route(): có "từ A" mà robot KHÔNG ở A -> đi tới A trước rồi mới tới đích.
  3. Robot còn trong dock -> undock (Nav2 không lái được robot đang đỗ).
  4. Gửi lần lượt từng điểm đích (goal hợp lệ của khu vực trong maps/zones.yaml) bằng action
     NavigateToPose của Nav2 (bt_navigator) – Nav2 lập đường + tránh vật cản. Lỗi -> thử lại max_retries lần.
  5. Câu lệnh mới khi đang chạy -> huỷ lộ trình cũ, chạy lộ trình mới. "Dừng lại" -> huỷ goal, robot dừng.

Hiển thị trên RViz2 (rviz/voice.rviz):
  /voice/zones          MarkerArray  ranh giới + tên + điểm đích các khu vực (khu vực đang tới tô đậm)
  /voice/route_markers  MarkerArray  thứ tự các điểm sẽ đi qua (1, 2, 3 ...)
  /voice/planned_route  nav_msgs/Path  đường dự kiến cả lộ trình (planner_server, action
                        ComputePathToPose, nối các chặng) – vẽ ngay khi nhận lệnh
Phản hồi: /voice/feedback (std_msgs/String) + log; speak:=true đọc phản hồi bằng espeak-ng.
Kết quả mỗi phiên: ~/tb4_results/voice_<thời_gian>/voice_run.json (câu lệnh, chặng, quỹ đạo) – vẽ:
  ros2 run tb4_lab04 zone_segmentation.py --run ~/tb4_results/voice_<...>/voice_run.json

Tham số (--ros-args -p tên:=giá_trị):
  zones_file        ''      -> maps/zones.yaml của package
  text_topic        voice/text
  sim_time          true    dùng giờ mô phỏng (/clock)
  undock            true    robot ở dock -> undock trước khi đi
  goal_timeout      240.0   giây (giờ mô phỏng) tối đa mỗi chặng
  max_retries       1       số lần thử lại một chặng thất bại
  pause_at_waypoint 1.0     giây dừng ở điểm trung gian
  preview_route     true    tính và vẽ đường dự kiến (ComputePathToPose)
  speak             false   đọc phản hồi bằng espeak-ng (tiếng Việt)
  base_frame        base_link
  results_dir       ''      -> ~/tb4_results/voice_<thời_gian>
"""

import json
import math
import os
import shutil
import subprocess
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
import lab_common as lc  # noqa: E402
from voice_command import ZoneCatalog, parse_command, plan_route, strip_accents  # noqa: E402
from zone_segmentation import PALETTE, ZoneMap  # noqa: E402

STATUS_NAMES = {4: 'SUCCEEDED', 5: 'CANCELED', 6: 'ABORTED'}


# ------------------------------------------------------------------ logic thuần (có unit test)
def leg_yaw(zone_goal, from_xy):
    """Hướng khi tới đích: yaw cố định trong zones.yaml, nếu không = hướng từ điểm trước tới đích."""
    if zone_goal.get('yaw') is not None:
        return float(zone_goal['yaw'])
    if from_xy is None:
        return 0.0
    dx, dy = zone_goal['x'] - from_xy[0], zone_goal['y'] - from_xy[1]
    if math.hypot(dx, dy) < 1e-3:
        return 0.0
    return math.atan2(dy, dx)


def hex_rgb(color):
    color = color.lstrip('#')
    return tuple(int(color[k:k + 2], 16) / 255.0 for k in (0, 2, 4))


def describe_route(catalog, legs, current):
    here = catalog.name(current) if current else 'vị trí hiện tại'
    return ' -> '.join([here] + [catalog.name(z) for z in legs])


# ------------------------------------------------------------------ node
def build_node_class():
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.duration import Duration
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
    from rclpy.time import Time
    from geometry_msgs.msg import Point, PoseStamped, PoseWithCovarianceStamped
    from nav2_msgs.action import ComputePathToPose, NavigateToPose
    from nav_msgs.msg import Path
    from std_msgs.msg import String
    from visualization_msgs.msg import Marker, MarkerArray
    import tf2_ros
    try:
        from irobot_create_msgs.action import Undock
        from irobot_create_msgs.msg import DockStatus
    except ImportError:
        Undock = DockStatus = None

    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)

    class VoiceNavigator(Node):
        def __init__(self):
            super().__init__('tb4_voice_navigator')
            p = self.declare_parameter
            p('zones_file', '')
            p('text_topic', 'voice/text')
            p('sim_time', True)
            p('undock', True)
            p('goal_timeout', 240.0)
            p('max_retries', 1)
            p('pause_at_waypoint', 1.0)
            p('preview_route', True)
            p('speak', False)
            p('base_frame', 'base_link')
            p('results_dir', '')
            self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL,
                                           bool(self.param('sim_time')))])

            path = self.param('zones_file') or lc.find_config('maps', 'zones.yaml')[0]
            if not path or not os.path.isfile(path):
                raise RuntimeError('Không tìm thấy maps/zones.yaml – chạy trước: '
                                   'ros2 run tb4_lab04 zone_segmentation.py')
            self.zmap = ZoneMap(path)
            self.catalog = ZoneCatalog(self.zmap.zones)
            self.valid = {z['id'] for z in self.zmap.zones if z.get('valid') and z.get('goal')}
            self.get_logger().info('Đọc {} khu vực từ {}: {}'.format(
                len(self.zmap.zones), path, self.catalog.describe()))
            bad = [z['id'] for z in self.zmap.zones if z['id'] not in self.valid]
            if bad:
                self.get_logger().warn('Khu vực không có điểm đích hợp lệ (sẽ từ chối): {}'.format(bad))

            self.tf_buffer = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
            self.amcl_pose = None
            self.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', self._on_amcl, latched)
            self.is_docked = None
            if DockStatus is not None:
                self.create_subscription(DockStatus, 'dock_status', self._on_dock, qos_profile_sensor_data)
                self.undock_client = ActionClient(self, Undock, 'undock')
            else:
                self.undock_client = None
            self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
            self.path_client = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')

            self.pub_feedback = self.create_publisher(String, 'voice/feedback', 10)
            self.pub_zones = self.create_publisher(MarkerArray, 'voice/zones', latched)
            self.pub_route = self.create_publisher(MarkerArray, 'voice/route_markers', latched)
            self.pub_path = self.create_publisher(Path, 'voice/planned_route', latched)
            self.create_subscription(String, self.param('text_topic'), self._on_text, 10)

            self.mission = None           # dict: id, legs, idx, attempts, cmd, leg_start, results
            self.undock_handle = None     # undock đang chạy (để lệnh mới không undock lần 2, "dừng" huỷ được)
            self.undocking = False
            self.mission_seq = 0
            self.goal_handle = None
            self.pending_timer = None
            self.last_fb_log = 0.0
            self.trajectory = []
            self.events = []
            self.out_dir = None
            self.create_timer(1.0, self._tick)
            self.create_timer(0.5, self._record_pose)
            self.publish_zone_markers()
            self.say('Sẵn sàng nhận lệnh. ' + 'Các khu vực: ' + self.catalog.describe() + '.', speak=False,
                     record=False)

        # -------------------------------------------------------------- tiện ích
        def param(self, n):
            return self.get_parameter(n).value

        def now_sec(self):
            return self.get_clock().now().nanoseconds * 1e-9

        def say(self, text, speak=True, level='info', record=True):
            # rclpy cấm MỘT dòng lệnh log ở nhiều mức khác nhau ("Logger severity cannot be changed between
            # calls") -> mỗi mức một dòng gọi riêng.
            if level == 'warn':
                self.get_logger().warning(text)
            elif level == 'error':
                self.get_logger().error(text)
            else:
                self.get_logger().info(text)
            self.pub_feedback.publish(String(data=text))
            if record:
                self.log_event('feedback', text=text)
            if speak and self.param('speak') and shutil.which('espeak-ng'):
                try:
                    subprocess.Popen(['espeak-ng', '-v', 'vi', '-s', '150', text],
                                     stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                except OSError:
                    pass

        def _on_amcl(self, msg):
            q = msg.pose.pose.orientation
            self.amcl_pose = (msg.pose.pose.position.x, msg.pose.pose.position.y,
                              lc.yaw_from_quat(q.x, q.y, q.z, q.w))

        def _on_dock(self, msg):
            self.is_docked = bool(msg.is_docked)

        def robot_pose(self):
            """(x, y, yaw) trong frame map: TF (cập nhật liên tục) hoặc /amcl_pose; None nếu chưa có."""
            try:
                t = self.tf_buffer.lookup_transform(self.zmap.frame_id, self.param('base_frame'), Time())
                q = t.transform.rotation
                return (t.transform.translation.x, t.transform.translation.y,
                        lc.yaw_from_quat(q.x, q.y, q.z, q.w))
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException):
                return self.amcl_pose

        def current_zone(self):
            pose = self.robot_pose()
            return None if pose is None else self.zmap.zone_at(pose[0], pose[1])

        def make_pose(self, x, y, yaw):
            ps = PoseStamped()
            ps.header.frame_id = self.zmap.frame_id
            ps.header.stamp = self.get_clock().now().to_msg()
            ps.pose.position.x, ps.pose.position.y = float(x), float(y)
            qx, qy, qz, qw = lc.quat_from_yaw(yaw)
            ps.pose.orientation.x, ps.pose.orientation.y = qx, qy
            ps.pose.orientation.z, ps.pose.orientation.w = qz, qw
            return ps

        def log_event(self, kind, **data):
            data.update({'t': round(self.now_sec(), 2), 'event': kind})
            self.events.append(data)
            self._save()

        def _record_pose(self):
            pose = self.robot_pose()
            if pose is None:
                return
            t = round(self.now_sec(), 2)
            if self.trajectory:
                _, lx, ly, _ = self.trajectory[-1]
                if math.hypot(pose[0] - lx, pose[1] - ly) < 0.05:
                    return
            self.trajectory.append([t, round(pose[0], 3), round(pose[1], 3), round(pose[2], 3)])

        def _save(self):
            try:
                if self.out_dir is None:
                    self.out_dir = self.param('results_dir') or lc.results_dir('voice_' + lc.timestamp())
                    os.makedirs(self.out_dir, exist_ok=True)
                with open(os.path.join(self.out_dir, 'voice_run.json'), 'w', encoding='utf-8') as f:
                    json.dump({'zones_file': self.zmap.path, 'events': self.events,
                               'trajectory': self.trajectory}, f, ensure_ascii=False, indent=1)
            except OSError as exc:
                self.get_logger().warn('Không lưu được kết quả: {}'.format(exc))

        # -------------------------------------------------------------- marker
        def publish_zone_markers(self, active=None, visited=()):
            arr = MarkerArray()
            clear = Marker()
            clear.action = Marker.DELETEALL
            arr.markers.append(clear)
            for k, z in enumerate(self.zmap.zones):
                r, g, b = hex_rgb(PALETTE[k % len(PALETTE)])
                is_active = z['id'] == active
                for n, poly in enumerate(z.get('polygons') or []):
                    m = Marker()
                    m.header.frame_id = self.zmap.frame_id
                    m.ns = 'zone_outline'
                    m.id = k * 100 + n
                    m.type = Marker.LINE_STRIP
                    m.action = Marker.ADD
                    m.pose.orientation.w = 1.0
                    m.scale.x = 0.12 if is_active else 0.05
                    m.color.r, m.color.g, m.color.b, m.color.a = r, g, b, 1.0 if is_active else 0.8
                    m.points = [Point(x=float(px), y=float(py), z=0.02) for px, py in poly + poly[:1]]
                    arr.markers.append(m)
                goal = z.get('goal')
                anchor = (goal['x'], goal['y']) if goal else tuple(z['center'])
                t = Marker()
                t.header.frame_id = self.zmap.frame_id
                t.ns = 'zone_label'
                t.id = k
                t.type = Marker.TEXT_VIEW_FACING
                t.action = Marker.ADD
                t.pose.position.x, t.pose.position.y, t.pose.position.z = anchor[0], anchor[1] + 0.45, 0.6
                t.pose.orientation.w = 1.0
                t.scale.z = 0.55 if is_active else 0.42
                t.color.r = t.color.g = t.color.b = 0.05
                t.color.a = 1.0
                # Font của RViz2 chỉ có ký tự Latin cơ bản -> bỏ dấu tiếng Việt ("Phong hoc (A)")
                t.text = strip_accents(self.catalog.name(z['id']) if z['name'] != 'Khu vực ' + z['id']
                                       else z['id'])
                if z['id'] in visited:
                    t.text += ' [OK]'
                arr.markers.append(t)
                if goal:
                    s = Marker()
                    s.header.frame_id = self.zmap.frame_id
                    s.ns = 'zone_goal'
                    s.id = k
                    s.type = Marker.CYLINDER
                    s.action = Marker.ADD
                    s.pose.position.x, s.pose.position.y, s.pose.position.z = goal['x'], goal['y'], 0.05
                    s.pose.orientation.w = 1.0
                    s.scale.x = s.scale.y = 0.35 if is_active else 0.22
                    s.scale.z = 0.1
                    s.color.r, s.color.g, s.color.b, s.color.a = r, g, b, 1.0
                    arr.markers.append(s)
            self.pub_zones.publish(arr)

        def publish_route_markers(self, legs, start_xy):
            arr = MarkerArray()
            clear = Marker()
            clear.action = Marker.DELETEALL
            arr.markers.append(clear)
            pts = ([start_xy] if start_xy else []) + [
                (self.zmap.by_id[z]['goal']['x'], self.zmap.by_id[z]['goal']['y']) for z in legs]
            if len(pts) >= 2:
                line = Marker()
                line.header.frame_id = self.zmap.frame_id
                line.ns = 'route_order'
                line.id = 0
                line.type = Marker.LINE_STRIP
                line.action = Marker.ADD
                line.pose.orientation.w = 1.0
                line.scale.x = 0.04
                line.color.r, line.color.g, line.color.b, line.color.a = 0.85, 0.1, 0.1, 0.6
                line.points = [Point(x=float(x), y=float(y), z=0.05) for x, y in pts]
                arr.markers.append(line)
            for n, z in enumerate(legs):
                g = self.zmap.by_id[z]['goal']
                t = Marker()
                t.header.frame_id = self.zmap.frame_id
                t.ns = 'route_step'
                t.id = n
                t.type = Marker.TEXT_VIEW_FACING
                t.action = Marker.ADD
                t.pose.position.x, t.pose.position.y, t.pose.position.z = g['x'], g['y'] - 0.45, 0.6
                t.pose.orientation.w = 1.0
                t.scale.z = 0.5
                t.color.r, t.color.g, t.color.b, t.color.a = 0.85, 0.1, 0.1, 1.0
                t.text = '{}'.format(n + 1)
                arr.markers.append(t)
            self.pub_route.publish(arr)
            path = Path()
            path.header.frame_id = self.zmap.frame_id
            path.header.stamp = self.get_clock().now().to_msg()
            path.poses = [self.make_pose(x, y, 0.0) for x, y in pts]       # tạm: đường thẳng nối các điểm
            self.pub_path.publish(path)

        # -------------------------------------------------------------- xem trước đường đi (planner)
        def preview(self, mission):
            if not self.param('preview_route') or not self.path_client.server_is_ready():
                return
            pose = self.robot_pose()
            mission['preview'] = []
            self._preview_leg(mission, 0, pose)

        def _preview_leg(self, mission, n, prev):
            if self.mission is not mission or n >= len(mission['legs']):
                if self.mission is mission and mission.get('preview'):
                    path = Path()
                    path.header.frame_id = self.zmap.frame_id
                    path.header.stamp = self.get_clock().now().to_msg()
                    for seg in mission['preview']:
                        path.poses.extend(seg)
                    self.pub_path.publish(path)
                    length = sum(math.hypot(b.pose.position.x - a.pose.position.x,
                                            b.pose.position.y - a.pose.position.y)
                                 for seg in mission['preview'] for a, b in zip(seg, seg[1:]))
                    self.get_logger().info('Đường dự kiến: {:.1f} m qua {} chặng (/voice/planned_route)'.format(
                        length, len(mission['preview'])))
                    mission['planned_length'] = round(length, 2)
                return
            g = self.zmap.by_id[mission['legs'][n]]['goal']
            goal = ComputePathToPose.Goal()
            goal.goal = self.make_pose(g['x'], g['y'], leg_yaw(g, prev[:2] if prev else None))
            if n > 0 and prev is not None:
                goal.start = self.make_pose(prev[0], prev[1], prev[2] if len(prev) > 2 else 0.0)
                goal.use_start = True
            else:
                goal.use_start = False
            fut = self.path_client.send_goal_async(goal)

            def on_accept(f):
                h = f.result()
                if h is None or not h.accepted:
                    return
                h.get_result_async().add_done_callback(on_result)

            def on_result(f):
                res = f.result()
                if res is not None and res.status == 4 and res.result.path.poses:
                    mission['preview'].append(list(res.result.path.poses))
                nxt = (g['x'], g['y'], leg_yaw(g, prev[:2] if prev else None))
                self._preview_leg(mission, n + 1, nxt)
            fut.add_done_callback(on_accept)

        # -------------------------------------------------------------- nhận câu lệnh
        def _on_text(self, msg):
            text = msg.data.strip()
            if not text:
                return
            try:
                self.handle_text(text)
            except Exception as exc:  # noqa: BLE001 – một câu lệnh lỗi không được làm sập node đang lái robot
                self.get_logger().error('Lỗi khi xử lý "{}": {}: {}'.format(text, type(exc).__name__, exc))
                self.pub_feedback.publish(String(data='Lỗi nội bộ khi xử lý câu lệnh – xem log.'))

        def handle_text(self, text):
            cmd = parse_command(text, self.catalog)
            self.get_logger().info('Câu lệnh: "{}" -> {}'.format(text, cmd.intent))
            self.log_event('command', text=text, intent=cmd.intent, start=cmd.start, targets=cmd.targets,
                           unknown=cmd.unknown)
            if cmd.intent == 'stop':
                if self.mission is None:
                    self.say('Robot đang đứng yên.')
                else:
                    self.abort_mission('Đã dừng theo lệnh.', 'CANCELED')
                return
            if cmd.intent == 'list':
                self.say(cmd.message)
                return
            if cmd.intent == 'where':
                z = self.current_zone()
                self.say('Robot đang ở {}.'.format(self.catalog.name(z)) if z else
                         'Chưa xác định được vị trí robot (chưa có TF map -> base_link).')
                return
            if cmd.intent != 'navigate':
                self.say(cmd.message + (' Robot giữ nguyên nhiệm vụ hiện tại.' if self.mission else ''),
                         level='warn')
                return
            bad = [z for z in [cmd.start] + cmd.targets if z is not None and z not in self.valid]
            if bad:
                self.say('Khu vực {} không có điểm đích hợp lệ – không thể đi.'.format(
                    ', '.join(self.catalog.name(z) for z in bad)), level='warn')
                return
            here = self.current_zone()
            legs = plan_route(cmd, here)
            if self.mission is not None:
                self.abort_mission('Nhận lệnh mới – huỷ lộ trình cũ.', 'PREEMPTED', announce=False)
            self.mission_seq += 1
            pose = self.robot_pose()
            mission = {'id': self.mission_seq, 'text': text, 'legs': legs, 'idx': 0, 'attempts': 0,
                       'results': [], 'start_zone': here, 't0': self.now_sec()}
            self.mission = mission
            note = ''
            if legs == [here]:
                note = ' (robot đã ở trong {} – đi tới điểm đích của khu vực)'.format(self.catalog.name(here))
            if cmd.start and cmd.start != here:
                note = ' (robot đang ở {} nên tới {} trước)'.format(
                    self.catalog.name(here) if here else 'ngoài các khu vực', self.catalog.name(cmd.start))
            self.say('Lộ trình: {}{}.'.format(describe_route(self.catalog, legs, here), note))
            self.log_event('route', mission=mission['id'], legs=legs, here=here)
            self.publish_route_markers(legs, pose[:2] if pose else None)
            self.publish_zone_markers(active=legs[0])
            self.preview(mission)
            if self.undocking:
                self.get_logger().info('Đang undock – chạy lộ trình mới ngay sau khi undock xong.')
            elif self.param('undock') and self.is_docked:
                self.start_undock(mission)
            else:
                self.send_leg(mission)

        # -------------------------------------------------------------- undock
        def start_undock(self, mission):
            if self.undock_client is None or not self.undock_client.wait_for_server(timeout_sec=2.0):
                self.say('Không có action /undock – thử đi luôn.', level='warn')
                self.send_leg(mission)
                return
            self.say('Robot đang ở dock – undock trước.', speak=False)
            self.undocking = True
            fut = self.undock_client.send_goal_async(Undock.Goal())

            def on_accept(f):
                h = f.result()
                if h is None or not h.accepted:
                    self.undocking = False
                    if self.mission is not None:
                        self.abort_mission('Undock bị từ chối.', 'ABORTED')
                    return
                self.undock_handle = h
                h.get_result_async().add_done_callback(on_result)

            def on_result(f):
                self.undocking = False
                self.undock_handle = None
                res = f.result()
                cur = self.mission              # có thể là lộ trình MỚI nhận trong lúc undock
                if cur is None:
                    return
                if res is None or res.status != 4:
                    self.abort_mission('Undock thất bại (status {}).'.format(
                        None if res is None else res.status), 'ABORTED')
                    return
                self.get_logger().info('Undock xong.')
                self.send_leg(cur)
            fut.add_done_callback(on_accept)

        # -------------------------------------------------------------- chạy từng chặng
        def send_leg(self, mission):
            if self.mission is not mission:
                return
            if mission['idx'] >= len(mission['legs']):
                self.finish_mission(mission)
                return
            if not self.nav_client.wait_for_server(timeout_sec=2.0):
                self.abort_mission('Nav2 chưa sẵn sàng (không có action navigate_to_pose).', 'ABORTED')
                return
            zid = mission['legs'][mission['idx']]
            z = self.zmap.by_id[zid]
            pose = self.robot_pose()
            yaw = leg_yaw(z['goal'], pose[:2] if pose else None)
            goal = NavigateToPose.Goal()
            goal.pose = self.make_pose(z['goal']['x'], z['goal']['y'], yaw)
            mission['attempts'] += 1
            mission['leg_start'] = self.now_sec()
            mission['leg_from'] = pose[:2] if pose else None
            done = [r['zone'] for r in mission['results'] if r['status'] == 'SUCCEEDED']
            self.publish_zone_markers(active=zid, visited=done)
            self.say('Chặng {}/{}: đi tới {} ({:.2f}, {:.2f}){}.'.format(
                mission['idx'] + 1, len(mission['legs']), self.catalog.name(zid), z['goal']['x'],
                z['goal']['y'], '' if mission['attempts'] == 1 else ' – thử lại lần {}'.format(
                    mission['attempts'] - 1)), speak=mission['attempts'] == 1)
            fut = self.nav_client.send_goal_async(goal, feedback_callback=self._on_nav_feedback)

            def on_accept(f):
                h = f.result()
                if self.mission is not mission:
                    if h is not None and h.accepted:
                        h.cancel_goal_async()
                    return
                if h is None or not h.accepted:
                    self.leg_done(mission, 'REJECTED')
                    return
                self.goal_handle = h
                h.get_result_async().add_done_callback(
                    lambda rf: self.leg_done(mission, STATUS_NAMES.get(
                        rf.result().status if rf.result() is not None else -1, 'UNKNOWN'), h))
            fut.add_done_callback(on_accept)

        def _on_nav_feedback(self, msg):
            fb = msg.feedback
            if time.monotonic() - self.last_fb_log > 5.0 and self.mission is not None:
                self.last_fb_log = time.monotonic()
                eta = Duration.from_msg(fb.estimated_time_remaining).nanoseconds * 1e-9
                self.get_logger().info('  -> {}: còn {:.1f} m | ETA ~{:.0f} s | recovery {}'.format(
                    self.mission['legs'][self.mission['idx']], fb.distance_remaining, eta,
                    fb.number_of_recoveries))

        def leg_done(self, mission, status, handle=None):
            if self.mission is not mission:
                return
            if handle is not None and handle is not self.goal_handle:
                return
            self.goal_handle = None
            zid = mission['legs'][mission['idx']]
            pose = self.robot_pose()
            g = self.zmap.by_id[zid]['goal']
            err = None if pose is None else round(math.hypot(pose[0] - g['x'], pose[1] - g['y']), 3)
            dur = round(self.now_sec() - mission['leg_start'], 1)
            arrived_zone = None if pose is None else self.zmap.zone_at(pose[0], pose[1])
            if status == 'SUCCEEDED':
                mission['results'].append({'zone': zid, 'status': status, 'duration': dur, 'error_xy': err,
                                           'attempts': mission['attempts'], 'arrived_zone': arrived_zone})
                self.log_event('leg', mission=mission['id'], zone=zid, status=status, duration=dur,
                               error_xy=err, arrived_zone=arrived_zone, attempts=mission['attempts'])
                self.say('Đã tới {} sau {:.0f} s (sai số {} m).'.format(
                    self.catalog.name(zid), dur, '?' if err is None else '{:.2f}'.format(err)))
                mission['idx'] += 1
                mission['attempts'] = 0
                if mission['idx'] < len(mission['legs']):
                    self._after(float(self.param('pause_at_waypoint')), lambda: self.send_leg(mission))
                else:
                    self.finish_mission(mission)
                return
            if status in ('ABORTED', 'REJECTED', 'TIMEOUT', 'UNKNOWN') and \
                    mission['attempts'] <= int(self.param('max_retries')):
                self.say('Chặng tới {} thất bại ({}) – thử lại.'.format(self.catalog.name(zid), status),
                         level='warn', speak=False)
                self._after(1.0, lambda: self.send_leg(mission))
                return
            mission['results'].append({'zone': zid, 'status': status, 'duration': dur, 'error_xy': err,
                                       'attempts': mission['attempts'], 'arrived_zone': arrived_zone})
            self.log_event('leg', mission=mission['id'], zone=zid, status=status, duration=dur,
                           error_xy=err, arrived_zone=arrived_zone, attempts=mission['attempts'])
            self.abort_mission('Không tới được {} ({}).'.format(self.catalog.name(zid), status), status,
                               cancel=False)

        def _after(self, sec, fn):
            if self.pending_timer is not None:
                self.pending_timer.cancel()
                self.destroy_timer(self.pending_timer)
            holder = {}

            def once():
                t = holder.pop('t', None)
                if t is not None:
                    t.cancel()
                    self.destroy_timer(t)
                if self.pending_timer is t:
                    self.pending_timer = None
                fn()
            holder['t'] = self.pending_timer = self.create_timer(max(0.01, sec), once)

        def _tick(self):
            m = self.mission
            if m is None or self.goal_handle is None or 'leg_start' not in m:
                return
            if self.now_sec() - m['leg_start'] > float(self.param('goal_timeout')):
                self.say('Quá {:.0f} s – huỷ chặng hiện tại.'.format(self.param('goal_timeout')), level='warn',
                         speak=False)
                h = self.goal_handle
                self.goal_handle = None
                h.cancel_goal_async()
                self.leg_done(m, 'TIMEOUT')

        def finish_mission(self, mission):
            ok = all(r['status'] == 'SUCCEEDED' for r in mission['results'])
            dur = self.now_sec() - mission['t0']
            self.log_event('mission', mission=mission['id'], status='SUCCEEDED' if ok else 'FAILED',
                           duration=round(dur, 1), legs=mission['legs'], results=mission['results'],
                           planned_length=mission.get('planned_length'))
            self.mission = None
            self.publish_zone_markers(visited=[r['zone'] for r in mission['results']])
            self.say('Hoàn thành lộ trình tới {} sau {:.0f} s.'.format(
                self.catalog.name(mission['legs'][-1]), dur))

        def abort_mission(self, reason, status, announce=True, cancel=True):
            mission = self.mission
            self.mission = None
            if self.pending_timer is not None:
                self.pending_timer.cancel()
                self.destroy_timer(self.pending_timer)
                self.pending_timer = None
            if cancel and self.goal_handle is not None:
                self.goal_handle.cancel_goal_async()
            self.goal_handle = None
            # KHÔNG huỷ undock đang chạy: dừng giữa chừng để robot nửa trong nửa ngoài dock -> dock (vật cản trên
            # costmap, dock_obstacle.py) đè lên robot, Nav2 không lập được đường, spin báo "Collision Ahead" (đã
            # gặp khi kiểm thử). Undock chỉ lùi ~0,4 m + quay; xong thì robot đứng yên vì không còn lộ trình.
            if self.undocking and status not in ('PREEMPTED',):
                reason += ' Robot lùi nốt ra khỏi dock (~0,4 m) rồi đứng yên.'
            if mission is not None:
                self.log_event('mission', mission=mission['id'], status=status, reason=reason,
                               legs=mission['legs'], results=mission['results'])
            self.publish_zone_markers()
            if announce:
                self.say(reason, level='warn' if status != 'CANCELED' else 'info')

        def shutdown(self):
            if self.goal_handle is not None:
                fut = self.goal_handle.cancel_goal_async()
                rclpy.spin_until_future_complete(self, fut, timeout_sec=3.0)
            self._save()

    return VoiceNavigator


def main():
    import signal
    import rclpy
    try:
        from rclpy.signals import SignalHandlerOptions
        rclpy.init(args=sys.argv, signal_handler_options=SignalHandlerOptions.NO)
    except (ImportError, TypeError):
        rclpy.init(args=sys.argv)
    signal.signal(signal.SIGTERM, lambda signum, frame: (_ for _ in ()).throw(KeyboardInterrupt()))
    node = None
    try:
        try:
            node = build_node_class()()
        except (RuntimeError, OSError, ValueError) as exc:
            print('[tb4_lab04] LỖI: {}'.format(exc), flush=True)
            return 2
        rclpy.spin(node)
        return 0
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        if node is not None:
            try:
                if rclpy.ok():
                    node.shutdown()
                if node.out_dir:
                    print('[tb4_lab04] Kết quả: {}/voice_run.json'.format(node.out_dir), flush=True)
                node.destroy_node()
            except Exception:  # noqa: BLE001
                pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    lc.install_ctrl_c_handlers()
    sys.exit(lc.run_main(main, interrupted_code=0))
