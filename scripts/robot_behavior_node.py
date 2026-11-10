#!/usr/bin/env python3
"""Bài 6 – Phần 3: node điều khiển hành vi TurtleBot 4 theo cử chỉ tay.

  ros2 run tb4_lab04 robot_behavior_node.py
  ros2 launch tb4_lab04 gesture.launch.py           # cùng gesture_recognition_node

Subscribe /detected_gesture (std_msgs/String, nhãn ỔN ĐỊNH do gesture_recognition_node gửi mỗi khung)
-> BehaviorStateMachine (gesture_logic.py) -> SegmentRunner -> publish /cmd_vel (geometry_msgs/Twist).

  Cử chỉ              Hành vi
  Wave (vẫy tay)      chọn "chào lại": xoay trái-phải (mặc định cần ngón cái xác nhận, xem
                      wave_requires_confirm)
  Pointing_Up         chọn "tiến-lùi 4 lần"
  Victory             chọn "quỹ đạo tròn"
  Thumb_Up            XÁC NHẬN -> thực hiện hành vi đã chọn
  Open_Palm           DỪNG NGAY (giữ lựa chọn: ngón cái = chạy lại từ đầu)
  Closed_Fist         HUỶ hành vi + xoá lựa chọn -> trạng thái chờ

An toàn:
  * chỉ chạy sau cử chỉ xác nhận; chỉ phản ứng khi nhãn ổn định ĐỔI (sườn) + cooldown mỗi cử chỉ;
  * watchdog: không nhận /detected_gesture quá signal_timeout (1 s) -> dừng, huỷ hành vi;
  * LiDAR /scan: có vật cản gần hơn obstacle_stop_distance theo hướng đang chạy -> dừng;
  * robot không nhúc nhích (kẹt) hoặc quá thời gian -> dừng; Ctrl+C -> gửi Twist 0 trước khi thoát;
  * khởi động khi robot đang ở dock: tự undock rồi chạy thẳng setup_forward (1 m) ra giữa sân để mọi
    hành vi nằm trong vùng trống (bán kính ~1.1 m quanh điểm bắt đầu).

Topic: pub /cmd_vel, /robot_behavior/state (JSON); sub /detected_gesture, /odom, /scan, /dock_status;
action client /undock (irobot_create_msgs).
Kết quả: ~/tb4_results/gesture_<thời_gian>/events.csv, odom.csv (vẽ: plot_gesture_run.py).
"""

import csv
import json
import math
import os
import signal
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
import gesture_logic as gl  # noqa: E402


def main(argv=None):
    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rclpy.clock import Clock, ClockType
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.signals import SignalHandlerOptions
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import String

    try:
        from irobot_create_msgs.action import Undock
        from irobot_create_msgs.msg import DockStatus
    except ImportError:  # không phải Create 3 / chưa cài -> bỏ qua bước undock
        Undock = DockStatus = None

    class BehaviorNode(Node):
        def __init__(self):
            super().__init__('robot_behavior_node')
            P = self._param
            self.prm = {
                'gesture_topic': P('gesture_topic', '/detected_gesture'),
                'cmd_vel_topic': P('cmd_vel_topic', '/cmd_vel'),
                'odom_topic': P('odom_topic', '/odom'),
                'scan_topic': P('scan_topic', '/scan'),
                'state_topic': P('state_topic', '/robot_behavior/state'),
                'control_rate': float(P('control_rate', 20.0)),
                'signal_timeout': float(P('signal_timeout', 1.0)),
                'selection_timeout': float(P('selection_timeout', 15.0)),
                'cooldown': float(P('cooldown', 1.0)),
                'wave_requires_confirm': gl.as_bool(P('wave_requires_confirm', True)),
                'odom_timeout': float(P('odom_timeout', 1.0)),
                'max_lin_acc': float(P('max_lin_acc', 0.5)),
                'max_ang_acc': float(P('max_ang_acc', 2.0)),
                'stall_timeout': float(P('stall_timeout', 4.0)),
                'use_scan_safety': gl.as_bool(P('use_scan_safety', True)),
                'obstacle_stop_distance': float(P('obstacle_stop_distance', 0.40)),
                'obstacle_half_angle': float(P('obstacle_half_angle', 0.6)),
                'lidar_yaw': float(P('lidar_yaw', float('nan'))),
                'auto_undock': gl.as_bool(P('auto_undock', True)),
                'dock_wait': float(P('dock_wait', 8.0)),
                'undock_timeout': float(P('undock_timeout', 120.0)),
                'setup_forward': float(P('setup_forward', 1.0)),
                'setup_speed': float(P('setup_speed', 0.2)),
                'stop_burst': int(P('stop_burst', 5)),
                'save_results': gl.as_bool(P('save_results', True)),
            }
            self.bparams = {k: float(P(k, v)) for k, v in gl.DEFAULT_BEHAVIOR_PARAMS.items()}
            self.sm = gl.BehaviorStateMachine(self.prm['wave_requires_confirm'],
                                              self.prm['selection_timeout'], self.prm['cooldown'],
                                              self.prm['signal_timeout'])
            self.steady = Clock(clock_type=ClockType.STEADY_TIME)
            self.t0 = time.monotonic()
            self.pub_cmd = self.create_publisher(Twist, self.prm['cmd_vel_topic'], 10)
            self.pub_state = self.create_publisher(String, self.prm['state_topic'], 10)
            self.create_subscription(String, self.prm['gesture_topic'], self.on_gesture, 10)
            self.create_subscription(Odometry, self.prm['odom_topic'], self.on_odom, qos_profile_sensor_data)
            self.create_subscription(LaserScan, self.prm['scan_topic'], self.on_scan, qos_profile_sensor_data)
            self.pose = None
            self.pose_t = None
            # "Đồng hồ robot": tiến theo thời gian MÔ PHỎNG (stamp của /odom) khi có odom, theo đồng hồ thực khi
            # không có. Mọi thời gian của chuyển động (tăng/giảm tốc, phát hiện kẹt, giới hạn thời gian, undock)
            # tính bằng đồng hồ này -> máy yếu, Gazebo chạy chậm (real-time factor 0,3) không bị báo "quá thời
            # gian" / "kẹt" oan. Watchdog cử chỉ vẫn dùng đồng hồ thực (người ra cử chỉ theo thời gian thực).
            self.odom_stamp = None
            self.last_odom_stamp = None
            self.robot_t = 0.0
            self.last_tick_wall = None
            self.scan = None
            self.scan_t = None
            self.lidar_yaw = None if math.isnan(self.prm['lidar_yaw']) else self.prm['lidar_yaw']
            self.tf_buffer = None
            self.runner = None
            self.runner_kind = None          # 'behavior' | 'setup'
            self.zero_left = 0
            self.last_state_json = None
            self.last_gesture = gl.NONE
            self.warned_no_odom = False
            self.warned_no_scan = False
            self.finished_count = {}
            self.last_error_log = -1e9
            # bước chuẩn bị (undock + ra giữa sân)
            self.dock_status = None
            self.setup_phase = None
            self.setup_then = None           # hành vi chờ chạy sau khi undock (robot còn ở dock)
            # True khi robot ĐÃ ra giữa sân (undock + chạy thẳng xong) hoặc không cần (không khởi động ở dock).
            # Bước chuẩn bị bị huỷ giữa chừng (xoè tay / nắm tay / mất tín hiệu) -> False: robot có thể đang đứng
            # sát dock, quay mặt vào dock -> trước hành vi đầu tiên phải ra xa dock đã.
            self.setup_done = not self.prm['auto_undock']
            self.setup_t0 = None
            self.undock_client = None
            self.undock_goal = None
            self.undock_future = None
            self.undock_result_future = None
            if DockStatus is not None:
                self.create_subscription(DockStatus, '/dock_status', self.on_dock, qos_profile_sensor_data)
            if self.prm['auto_undock'] and Undock is not None:
                from rclpy.action import ActionClient
                self.undock_client = ActionClient(self, Undock, '/undock')
                self.setup_phase = 'wait_dock'
                self.setup_t0 = 0.0
                self.sm.begin_setup('Kiem tra dock...')
            elif self.prm['auto_undock']:
                self.get_logger().warn('Không có irobot_create_msgs – bỏ qua bước tự undock')
                self.setup_done = True
            if self.prm['use_scan_safety']:
                self._init_tf()
            self._init_results()
            period = 1.0 / max(1.0, self.prm['control_rate'])
            self.create_timer(period, self.control_tick, clock=self.steady)
            self.create_timer(0.2, self.publish_state, clock=self.steady)
            self._print_intro()

        # ------------------------------------------------------------ tiện ích
        def _param(self, name, default):
            # dynamic_typing: ghi 4 hay 4.0 trong YAML đều được (không lỗi InvalidParameterType)
            from rcl_interfaces.msg import ParameterDescriptor
            self.declare_parameter(name, default, ParameterDescriptor(dynamic_typing=True))
            value = self.get_parameter(name).value
            return default if value is None else value

        def now(self):
            return time.monotonic()

        def _init_tf(self):
            try:
                from tf2_ros.buffer import Buffer
                from tf2_ros.transform_listener import TransformListener
                self.tf_buffer = Buffer()
                self.tf_listener = TransformListener(self.tf_buffer, self)
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn('Không dùng được tf2 ({}) – dùng lidar_yaw mặc định'.format(exc))
                self.tf_buffer = None

        def _init_results(self):
            self.events_f = self.odom_f = None
            if not self.prm['save_results']:
                return
            try:
                root = os.path.expanduser(os.environ.get('TB4_RESULTS_DIR', '~/tb4_results'))
                self.results_dir = os.path.join(root, 'gesture_' + time.strftime('%Y%m%d_%H%M%S'))
                os.makedirs(self.results_dir, exist_ok=True)
                self.events_f = open(os.path.join(self.results_dir, 'events.csv'), 'w', newline='',
                                     encoding='utf-8')
                self.events_w = csv.writer(self.events_f)
                self.events_w.writerow(['t', 'type', 'gesture', 'state', 'selected', 'active', 'detail'])
                self.odom_f = open(os.path.join(self.results_dir, 'odom.csv'), 'w', newline='',
                                   encoding='utf-8')
                self.odom_w = csv.writer(self.odom_f)
                self.odom_w.writerow(['t', 'x', 'y', 'yaw', 'state', 'active', 'lin_cmd', 'ang_cmd'])
                self.last_odom_log = 0.0
            except OSError as exc:
                self.get_logger().warn('Không ghi được kết quả: {}'.format(exc))
                self.events_f = self.odom_f = None

        def record(self, kind, detail='', gesture=''):
            if self.events_f is None:
                return
            d = self.sm.describe()
            self.events_w.writerow(['{:.3f}'.format(self.now() - self.t0), kind, gesture, d['state'],
                                    d['selected'] or '', d['active'] or '', detail])
            self.events_f.flush()

        def _print_intro(self):
            lines = ['=== robot_behavior_node: điều khiển TurtleBot 4 bằng cử chỉ ===',
                     '  Wave        -> {} hành vi CHÀO (xoay trái-phải)'.format(
                         'chọn' if self.prm['wave_requires_confirm'] else 'thực hiện ngay'),
                     '  Pointing_Up -> chọn TIẾN-LÙI {} lần'.format(int(self.bparams['fb_repeats'])),
                     '  Victory     -> chọn QUỸ ĐẠO TRÒN r = {:.2f} m'.format(self.bparams['circle_radius']),
                     '  Thumb_Up    -> XÁC NHẬN, thực hiện hành vi đã chọn',
                     '  Open_Palm   -> DỪNG NGAY',
                     '  Closed_Fist -> HUỶ, về trạng thái chờ',
                     '  Mất /detected_gesture > {:.1f} s -> dừng'.format(self.prm['signal_timeout'])]
            for name in ('greet', 'forward_back', 'circle'):
                try:
                    plan = gl.build_plan(name, self.bparams)
                except ValueError as exc:
                    lines.append('  [{}] THAM SỐ SAI: {} – hành vi này sẽ không chạy'.format(name, exc))
                    continue
                r, end = gl.plan_extent(plan)
                lines.append('  [{}] {} đoạn, ~{:.0f} s, ra xa điểm xuất phát tối đa {:.2f} m, kết thúc lệch '
                             '{:.2f} m'.format(name, len(plan), sum(s.nominal_duration() for s in plan), r,
                                               math.hypot(end[0], end[1])))
            if self.events_f is not None:
                lines.append('  Kết quả: {}'.format(self.results_dir))
            self.get_logger().info('\n'.join(lines))

        # ------------------------------------------------------------ callback
        def on_gesture(self, msg):
            try:
                self._on_gesture(msg)
            except Exception as exc:  # noqa: BLE001
                self._callback_error('xử lý cử chỉ', exc)

        def _on_gesture(self, msg):
            label = gl.parse_gesture_message(msg.data)
            if label != self.last_gesture:
                self.record('gesture', gesture=label)
                self.last_gesture = label
            self.execute(self.sm.on_message(label, self.now()), gesture=label)

        def on_odom(self, msg):
            p = msg.pose.pose
            q = p.orientation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            self.pose = (p.position.x, p.position.y, yaw)
            self.pose_t = self.now()
            st = msg.header.stamp
            self.odom_stamp = st.sec + st.nanosec * 1e-9

        def on_scan(self, msg):
            self.scan = msg
            self.scan_t = self.now()

        def on_dock(self, msg):
            self.dock_status = bool(msg.is_docked)

        # ------------------------------------------------------------ thực thi action
        def execute(self, actions, gesture=''):
            for act in actions:
                kind = act[0]
                if kind == 'stop':
                    self.halt(act[1])
                elif kind == 'start':
                    self.start_behavior(act[1])
                elif kind == 'abort_setup':
                    self.abort_setup(act[1])
                elif kind == 'info':
                    self.get_logger().info(act[1])
                self.record(kind, act[1] if len(act) > 1 else '', gesture)

        def halt(self, reason, cancel_setup=True):
            was = self.runner is not None or self.setup_phase is not None
            if cancel_setup and self.setup_then is not None:     # đang undock trước khi chạy hành vi
                self._cancel_undock()
                self.setup_phase = None
                self.setup_then = None
            self.runner = None
            self.runner_kind = None
            self.zero_left = max(self.zero_left, self.prm['stop_burst'])
            self._publish(0.0, 0.0)
            self.get_logger().warn('DỪNG robot – {}{}'.format(reason, '' if was else ' (robot đang đứng yên)'))

        def start_behavior(self, name):
            try:
                plan = gl.build_plan(name, self.bparams)
            except ValueError as exc:
                self.get_logger().error(str(exc))
                self.execute(self.sm.on_behavior_finished(False, str(exc)))
                return
            if self.prm['auto_undock'] and self.undock_client is not None and not self.setup_done:
                if self.dock_status:
                    # Robot vẫn ở dock (bước chuẩn bị bị huỷ trước khi kịp rời dock): chạy tiến / vòng tròn sẽ
                    # đâm vào dock -> undock + ra giữa sân TRƯỚC, xong mới thực hiện hành vi đã xác nhận.
                    self.setup_then = name
                    self.setup_phase = 'wait_dock'
                    self.setup_t0 = self.robot_t
                    self.sm.message = 'Robot dang o dock -> undock truoc, roi thuc hien {}'.format(name)
                    self.get_logger().info('Robot đang ở dock -> undock + ra giữa sân trước khi thực hiện {}'.format(
                        name))
                    return
                pose = self.fresh_pose()
                if pose is not None:
                    # Bước chuẩn bị bị huỷ GIỮA CHỪNG: robot đã lùi khỏi dock nhưng có thể còn quay mặt vào dock.
                    # Dock ở gốc odom (robot khởi động trong dock) -> quay mặt ra xa gốc rồi chạy tới khi cách gốc
                    # SETUP_RADIUS, như sau một lần chuẩn bị bình thường.
                    plan_clear = self._clear_dock_plan(pose)
                    self.setup_then = name
                    self.setup_phase = 'forward'
                    self.setup_t0 = self.robot_t
                    self.sm.message = 'Ra xa dock truoc, roi thuc hien {}'.format(name)
                    self.get_logger().info('Bước chuẩn bị đã bị huỷ giữa chừng (robot ở ({:.2f}, {:.2f})) -> quay ra '
                                           'xa dock và ra giữa sân trước khi thực hiện {}'.format(pose[0], pose[1], name))
                    self.runner = gl.SegmentRunner(plan_clear, self.prm['max_lin_acc'], self.prm['max_ang_acc'],
                                                   stall_timeout=self.prm['stall_timeout'])
                    self.runner_kind = 'setup'
                    self.zero_left = 0
                    return
                self.get_logger().warn('Chưa ra giữa sân và không có /odom – thực hiện luôn (cẩn thận va dock)')
            self._start_runner(name, plan)

        def _clear_dock_plan(self, pose):
            """Quay mặt ra xa gốc odom (vị trí dock) rồi chạy thẳng tới khi cách gốc 0,35 m + setup_forward."""
            return gl.clear_dock_plan(pose, 0.35 + max(0.0, self.prm['setup_forward']),
                                      abs(self.prm['setup_speed']))

        def _start_runner(self, name, plan=None):
            plan = plan or gl.build_plan(name, self.bparams)
            self.runner = gl.SegmentRunner(plan, self.prm['max_lin_acc'], self.prm['max_ang_acc'],
                                           stall_timeout=self.prm['stall_timeout'])
            self.runner_kind = 'behavior'
            self.zero_left = 0
            self.get_logger().info('>>> THỰC HIỆN: {} – {} ({} đoạn, ~{:.0f} s)'.format(
                name, gl.BEHAVIOR_TEXT.get(name, name), len(plan), self.runner.total_nominal))

        def _cancel_undock(self):
            if self.undock_goal is not None:
                try:
                    self.undock_goal.cancel_goal_async()
                except Exception:  # noqa: BLE001
                    pass
            self.undock_goal = None
            self.undock_future = None

        def abort_setup(self, reason):
            self._cancel_undock()
            if self.runner_kind == 'setup':
                self.runner = None
                self.runner_kind = None
            self.setup_phase = None
            self.sm.end_setup(False, 'Huy buoc chuan bi ({})'.format(reason))
            self.get_logger().warn('Huỷ bước chuẩn bị: {}'.format(reason))

        def _finish_setup(self, ok, message, done=None):
            self.setup_phase = None
            if ok if done is None else done:
                self.setup_done = True
            then, self.setup_then = self.setup_then, None
            (self.get_logger().info if ok else self.get_logger().warn)(message)
            self.record('setup', message)
            if then is not None:                       # chuẩn bị xong -> chạy hành vi đã xác nhận
                if ok and self.sm.state == gl.EXECUTING and self.sm.active == then:
                    self._start_runner(then)
                elif not ok:
                    self.execute(self.sm.on_behavior_finished(False, message))
                return
            self.sm.end_setup(ok, message)

        def _publish(self, lin, ang):
            t = Twist()
            t.linear.x = float(lin)
            t.angular.z = float(ang)
            self.pub_cmd.publish(t)

        # ------------------------------------------------------------ an toàn LiDAR
        def _lidar_yaw(self):
            if self.lidar_yaw is not None:
                return self.lidar_yaw
            frame = self.scan.header.frame_id if self.scan is not None else ''
            if self.tf_buffer is not None and frame:
                try:
                    from rclpy.time import Time
                    tr = self.tf_buffer.lookup_transform('base_link', frame, Time())
                    q = tr.transform.rotation
                    self.lidar_yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
                    self.get_logger().info('LiDAR {} lệch {:.0f}° so với base_link (từ TF)'.format(
                        frame, math.degrees(self.lidar_yaw)))
                    return self.lidar_yaw
                except Exception:  # noqa: BLE001
                    pass
            return math.pi / 2.0 if 'rplidar' in frame else 0.0   # TurtleBot 4: rplidar quay +90°

        def obstacle(self, lin):
            if not self.prm['use_scan_safety'] or abs(lin) < 0.01:
                return False, None
            if self.scan is None or self.now() - self.scan_t > 1.0:
                if not self.warned_no_scan:
                    self.get_logger().warn('Chưa có /scan mới – chạy KHÔNG có kiểm tra vật cản LiDAR')
                    self.warned_no_scan = True
                return False, None
            s = self.scan
            return gl.obstacle_in_path(list(s.ranges), s.angle_min, s.angle_increment, s.range_min, s.range_max,
                                       self._lidar_yaw(), 1 if lin > 0 else -1,
                                       self.prm['obstacle_stop_distance'], self.prm['obstacle_half_angle'])

        # ------------------------------------------------------------ vòng điều khiển
        def fresh_pose(self):
            if self.pose is not None and self.now() - self.pose_t <= self.prm['odom_timeout']:
                return self.pose
            return None

        def _advance_robot_clock(self, now):
            wall_dt = 0.0 if self.last_tick_wall is None else max(0.0, now - self.last_tick_wall)
            self.last_tick_wall = now
            if self.fresh_pose() is not None and self.odom_stamp is not None:
                if self.last_odom_stamp is not None:
                    d = self.odom_stamp - self.last_odom_stamp
                    self.robot_t += d if 0.0 <= d < 5.0 else wall_dt   # stamp nhảy (reset sim) -> dùng giờ thực
                self.last_odom_stamp = self.odom_stamp
            else:
                self.robot_t += wall_dt
                self.last_odom_stamp = None
            return self.robot_t

        def control_tick(self):
            try:
                self._control_tick()
            except Exception as exc:  # noqa: BLE001 – không để một lỗi bất ngờ làm chết node (robot mất điều khiển)
                self._callback_error('vòng điều khiển', exc)

        def _callback_error(self, where, exc):
            now = self.now()
            if now - self.last_error_log > 5.0:
                self.last_error_log = now
                import traceback
                self.get_logger().error('Lỗi ở {}: {}: {}\n{}'.format(where, type(exc).__name__, exc,
                                                                      traceback.format_exc()))
            try:
                self.runner = None
                self.runner_kind = None
                self._publish(0.0, 0.0)
                self.zero_left = self.prm['stop_burst']
                if self.sm.state == gl.EXECUTING:
                    self.sm.on_behavior_finished(False, 'lỗi phần mềm: {}'.format(exc))
            except Exception:  # noqa: BLE001
                pass

        def _control_tick(self):
            now = self.now()
            rt = self._advance_robot_clock(now)
            self.execute(self.sm.tick(now))
            self._setup_tick(rt)
            if self.runner is not None:
                pose = self.fresh_pose()
                if pose is None and not self.warned_no_odom:
                    self.get_logger().warn('Không có /odom mới – đo chuyển động theo thời gian (kém chính xác)')
                    self.warned_no_odom = True
                lin, ang = self.runner.update(rt, pose)
                blocked, dist = self.obstacle(lin)
                kind = self.runner_kind
                if blocked or self.runner.error:
                    reason = self.runner.error or 'VẬT CẢN cách {:.2f} m phía {}'.format(
                        dist, 'trước' if lin > 0 else 'sau')
                    if kind == 'behavior':
                        self.execute(self.sm.on_behavior_finished(False, reason))   # -> ('stop', ...) -> halt
                        if self.runner is not None:
                            self.halt(reason)
                    else:
                        self.halt(reason, cancel_setup=False)
                        self._finish_setup(False, 'Dừng ra giữa sân: ' + reason)
                elif self.runner.done:
                    self.runner = None
                    self.runner_kind = None
                    self.zero_left = self.prm['stop_burst']
                    self._publish(0.0, 0.0)
                    if kind == 'behavior':
                        self.execute(self.sm.on_behavior_finished(True))
                    else:
                        self._finish_setup(True, 'Đã ra giữa sân – sẵn sàng nhận cử chỉ')
                else:
                    self._publish(lin, ang)
            elif self.zero_left > 0:
                self.zero_left -= 1
                self._publish(0.0, 0.0)
            self._log_odom(now)

        def _setup_tick(self, now):
            ph = self.setup_phase
            if ph is None:
                return
            expected = gl.EXECUTING if self.setup_then is not None else gl.SETUP
            if self.sm.state != expected:          # bị huỷ ở nơi khác (xoè tay, nắm tay, mất tín hiệu)
                self._cancel_undock()
                self.setup_phase = None
                self.setup_then = None
                return
            if ph == 'wait_dock':
                if self.dock_status is None:
                    if now - self.setup_t0 > self.prm['dock_wait']:
                        self._finish_setup(False, 'Không nhận /dock_status – bỏ qua undock', done=True)
                    return
                if not self.dock_status:
                    self._finish_setup(True, 'Robot không ở dock – bỏ qua undock, sẵn sàng nhận cử chỉ')
                    return
                if not self.undock_client.server_is_ready():
                    if now - self.setup_t0 > self.prm['dock_wait'] + 10.0:
                        self._finish_setup(False, 'Không thấy action /undock – bỏ qua')
                    return
                self.get_logger().info('Robot đang ở dock -> gửi /undock (lùi ra, quay 180°)')
                self.sm.message = 'Dang undock...'
                self.undock_future = self.undock_client.send_goal_async(Undock.Goal())
                self.setup_phase = 'undocking'
                self.setup_t0 = now
                return
            if ph == 'undocking':
                if now - self.setup_t0 > self.prm['undock_timeout']:
                    self.abort_setup('undock quá {:.0f} s'.format(self.prm['undock_timeout']))
                    return
                if self.undock_goal is None:
                    if self.undock_future is not None and self.undock_future.done():
                        gh = self.undock_future.result()
                        if gh is None or not gh.accepted:
                            self._finish_setup(False, 'Undock bị từ chối')
                            return
                        self.undock_goal = gh
                        self.undock_result_future = gh.get_result_async()
                    return
                if self.undock_result_future.done():
                    res = self.undock_result_future.result()
                    self.undock_goal = None
                    status = getattr(res, 'status', 0)
                    if status != 4:      # GoalStatus.STATUS_SUCCEEDED
                        self._finish_setup(False, 'Undock không thành công (status {}) – kiểm tra Gazebo'.format(
                            status))
                        return
                    d = self.prm['setup_forward']
                    if d <= 0.01:
                        self._finish_setup(True, 'Undock xong – sẵn sàng nhận cử chỉ')
                        return
                    self.get_logger().info('Undock xong -> chạy thẳng {:.2f} m ra giữa sân'.format(d))
                    self.sm.message = 'Ra giua san {:.1f} m...'.format(d)
                    self.runner = gl.SegmentRunner(
                        [gl.Segment('ra giua san', abs(self.prm['setup_speed']), 0.0, d, 'dist')],
                        self.prm['max_lin_acc'], self.prm['max_ang_acc'], stall_timeout=self.prm['stall_timeout'])
                    self.runner_kind = 'setup'
                    self.setup_phase = 'forward'
                return

        def _log_odom(self, now):
            if self.odom_f is None or self.pose is None or now - self.last_odom_log < 0.1:
                return
            self.last_odom_log = now
            d = self.sm.describe()
            lin, ang = self.runner.cmd if self.runner is not None else (0.0, 0.0)
            self.odom_w.writerow(['{:.3f}'.format(now - self.t0), '{:.4f}'.format(self.pose[0]),
                                  '{:.4f}'.format(self.pose[1]), '{:.4f}'.format(self.pose[2]), d['state'],
                                  'setup' if (self.runner_kind == 'setup' or self.setup_phase) else (d['active'] or ''),
                                  '{:.3f}'.format(lin),
                                  '{:.3f}'.format(ang)])

        def publish_state(self):
            d = self.sm.describe()
            d['progress'] = round(self.runner.overall_progress(), 2) if self.runner is not None else None
            d['gesture'] = self.last_gesture
            text = json.dumps(d, ensure_ascii=True)
            self.pub_state.publish(String(data=text))
            key = (d['state'], d['selected'], d['active'], d['message'])
            if key != self.last_state_json:
                self.last_state_json = key
                self.get_logger().info('[trạng thái] {:<11} chọn={:<12} chạy={:<12} {}'.format(
                    d['state'], d['selected'] or '-', d['active'] or '-', d['message']))

        def shutdown(self):
            for _ in range(3):
                self._publish(0.0, 0.0)
                time.sleep(0.02)
            if self.undock_goal is not None:
                try:
                    self.undock_goal.cancel_goal_async()
                except Exception:  # noqa: BLE001
                    pass
            for f in (self.events_f, self.odom_f):
                if f is not None:
                    try:
                        f.close()
                    except OSError:
                        pass

    rclpy.init(args=argv, signal_handler_options=SignalHandlerOptions.NO)
    stop = {'flag': False, 'count': 0}

    def on_signal(signum, frame):
        stop['count'] += 1
        stop['flag'] = True
        if stop['count'] >= 3:
            raise KeyboardInterrupt

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)
    node = BehaviorNode()
    code = 0
    try:
        while rclpy.ok() and not stop['flag']:
            rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt:
        pass
    except Exception as exc:  # noqa: BLE001
        node.get_logger().error('Lỗi: {}: {}'.format(type(exc).__name__, exc))
        code = 1
    finally:
        node.get_logger().info('Thoát – gửi Twist 0 để robot dừng')
        try:
            node.shutdown()
        finally:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
    return code


if __name__ == '__main__':
    sys.exit(main())
