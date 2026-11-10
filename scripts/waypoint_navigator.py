#!/usr/bin/env python3
"""Phần 3 – Navigation tự động BẰNG CODE: robot tự đi tuần tự Goal 1 -> Goal 2 -> Goal 3 bằng Nav2.

  ros2 run tb4_lab04 waypoint_navigator.py
  ros2 run tb4_lab04 waypoint_navigator.py --ros-args -p goals_file:=/duong/dan/goals.yaml \
      -p look_around:=true -p return_home:=true

Dùng thư viện nav2_simple_commander (lớp BasicNavigator -> action /navigate_to_pose của bt_navigator).
Sau khi node chạy, KHÔNG cần (và không được) chạm bàn phím/chuột: node tự
  1. chờ AMCL và bt_navigator ở trạng thái lifecycle "active" (có giới hạn thời gian, báo rõ thiếu gì);
  2. đặt initial pose cho AMCL – chỉ khi robot còn nằm trong dock (lúc đó chắc chắn đang ở điểm spawn).
     Robot đã rời dock (chạy lại node lần 2) thì GIỮ ước lượng hiện tại của AMCL, không reset sai;
  3. undock (action /undock của Create 3) – robot đang đỗ thì Nav2 không lái ra được;
  4. gửi từng goal (goals.yaml), in tiến độ (quãng đường còn lại, ETA, số recovery); goal thất bại thì xoá
     costmap và thử lại (max_retries); quá goal_timeout thì huỷ;
  5. (tuỳ chọn) xoay 360° tại goal để camera quét quanh (look_around), về chỗ sau undock (return_home),
     vào dock (dock_at_end);
  6. in bảng tổng kết + lưu ~/tb4_results/nav_<thời_gian>/run.json (quỹ đạo, đường Nav2 lập) và
     summary.csv – vẽ ảnh báo cáo: ros2 run tb4_lab04 check_goals.py --plot --run <run.json>.
Ctrl+C: huỷ goal đang chạy (robot dừng) rồi mới thoát. Mã thoát 0 = mọi goal SUCCEEDED.

Tham số (--ros-args -p tên:=giá_trị):
  goals_file         ''     -> config/goals.yaml của package (bản trong thư mục source)
  initial_pose_mode  auto   auto | always | never
  undock             true
  pause_at_goal      2.0    giây (giờ mô phỏng) dừng tại mỗi goal – để YOLO/video kịp ghi nhận
  look_around        false  xoay 360° tại mỗi goal (action /spin của Nav2)
  goal_timeout       300.0  giây (giờ mô phỏng) tối đa cho một goal
  max_retries        1      số lần thử lại một goal thất bại
  loops              1      số vòng đi hết danh sách goal
  return_home        false  cuối cùng quay về chỗ robot đứng sau khi undock
  dock_at_end        false  cuối cùng vào dock (cần return_home:=true)
  startup_timeout    300.0  giây (giờ thực) chờ Nav2/AMCL sẵn sàng
  results_dir        ''     -> ~/tb4_results/nav_<thời_gian>
"""

import csv
import json
import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
import lab_common as lc  # noqa: E402
from check_goals import load_goals  # noqa: E402

STATUS_TEXT = {'SUCCEEDED': 'SUCCEEDED', 'FAILED': 'ABORTED', 'CANCELED': 'CANCELED',
               'UNKNOWN': 'UNKNOWN', 'TIMEOUT': 'TIMEOUT', 'REJECTED': 'REJECTED'}
# Hiệp phương sai của initial pose (giống công cụ 2D Pose Estimate của RViz2). nav2_simple_commander
# gửi hiệp phương sai 0 -> mọi hạt AMCL trùng một điểm, lệch chút là khó tự sửa.
INIT_COV_XY = 0.25               # phương sai x, y (m^2) như RViz (độ lệch chuẩn 0.5 m)
INIT_COV_YAW = 0.0685            # phương sai yaw (rad^2) như RViz (~15°)


# ------------------------------------------------------------------ logic thuần (có unit test)
def decide_initial_pose(mode, is_docked):
    """(có đặt initial pose không, lý do). is_docked: True/False/None (chưa biết)."""
    mode = (mode or 'auto').strip().lower()
    if mode == 'always':
        return True, 'initial_pose_mode=always'
    if mode == 'never':
        return False, 'initial_pose_mode=never – giữ ước lượng hiện tại của AMCL'
    if mode != 'auto':
        raise ValueError('initial_pose_mode phải là auto | always | never (đang là "{}")'.format(mode))
    if is_docked is True:
        return True, 'robot đang ở trong dock = đúng điểm spawn của lúc làm bản đồ'
    if is_docked is False:
        return False, ('robot ĐÃ rời dock (không còn ở điểm spawn) – giữ ước lượng hiện tại của AMCL; '
                       'nếu sai hãy dùng 2D Pose Estimate trên RViz2')
    return True, 'không đọc được /dock_status – coi như robot còn ở điểm spawn'


def pose_std(cov):
    """Độ lệch chuẩn (x, y, yaw) từ hiệp phương sai 6x6 (36 phần tử) của PoseWithCovariance."""
    return (math.sqrt(max(0.0, cov[0])), math.sqrt(max(0.0, cov[7])), math.sqrt(max(0.0, cov[35])))


def path_length(points):
    return sum(math.hypot(points[k + 1][0] - points[k][0], points[k + 1][1] - points[k][1])
               for k in range(len(points) - 1))


def summary_lines(results):
    lines = ['{:<4} {:<10} {:>8} {:>9} {:>9} {:>9} {:>4}  {}'.format(
        '#', 'kết quả', 'thời gian', 'quãng đg', 'sai số xy', 'sai số yaw', 'thử', 'tên')]
    for r in results:
        err = '-' if r.get('error_xy') is None else '{:.2f} m'.format(r['error_xy'])
        eyaw = '-' if r.get('error_yaw') is None else '{:.0f}°'.format(math.degrees(r['error_yaw']))
        lines.append('{:<4} {:<10} {:>7.1f}s {:>7.1f} m {:>9} {:>9} {:>4}  {}'.format(
            r['label'], r['status'], r.get('duration', 0.0), r.get('distance', 0.0), err, eyaw,
            r.get('attempts', 1), r['name']))
    ok = sum(1 for r in results if r['status'] == 'SUCCEEDED' and not r.get('extra'))
    total = sum(1 for r in results if not r.get('extra'))
    lines.append('Goal thành công: {}/{}'.format(ok, total))
    return lines, ok, total


# ------------------------------------------------------------------ node
def build_node_class():
    """Tạo lớp node (import ROS bên trong để unit test phần thuần không cần ROS)."""
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.duration import Duration
    from rclpy.parameter import Parameter
    from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy,
                           qos_profile_sensor_data)
    from action_msgs.msg import GoalStatus
    from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
    from lifecycle_msgs.srv import GetState
    from nav_msgs.msg import Path
    from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
    from visualization_msgs.msg import Marker, MarkerArray
    try:
        from irobot_create_msgs.action import Dock, Undock
        from irobot_create_msgs.msg import DockStatus
    except ImportError:          # không phải Create 3 / chưa cài irobot_create_msgs
        Dock = Undock = DockStatus = None

    class TB4WaypointNavigator(BasicNavigator):
        def __init__(self):
            super().__init__(node_name='tb4_waypoint_navigator')
            self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])
            p = self.declare_parameter
            p('goals_file', '')
            p('initial_pose_mode', 'auto')
            p('undock', True)
            p('pause_at_goal', 2.0)
            p('look_around', False)
            p('goal_timeout', 300.0)
            p('max_retries', 1)
            p('loops', 1)
            p('return_home', False)
            p('dock_at_end', False)
            p('startup_timeout', 300.0)
            p('results_dir', '')

            self.is_docked = None
            self.amcl = None              # (x, y, yaw, cov, stamp_sec)
            self.amcl_count = 0
            self.trajectory = []          # [t, x, y, yaw]
            self.first_plan = None        # đường đi ĐẦU TIÊN planner lập cho goal hiện tại
            self.capture_plan = False
            self.goal_states = []
            self.goals = []

            amcl_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                  reliability=ReliabilityPolicy.RELIABLE)
            self.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', self._on_amcl, amcl_qos)
            self.create_subscription(Path, 'plan', self._on_plan, 10)
            if DockStatus is not None:
                self.create_subscription(DockStatus, 'dock_status', self._on_dock,
                                         qos_profile_sensor_data)
                self.undock_client = ActionClient(self, Undock, 'undock')
                self.dock_client = ActionClient(self, Dock, 'dock')
            else:
                self.undock_client = self.dock_client = None
            self.init_pub = self.create_publisher(PoseWithCovarianceStamped, 'initialpose', 10)
            marker_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                    reliability=ReliabilityPolicy.RELIABLE)
            self.marker_pub = self.create_publisher(MarkerArray, 'nav_goals/markers', marker_qos)
            self.task_active = False

        # -------------------------------------------------------------- callback
        def _on_dock(self, msg):
            self.is_docked = bool(msg.is_docked)

        def _on_amcl(self, msg):
            q = msg.pose.pose.orientation
            yaw = lc.yaw_from_quat(q.x, q.y, q.z, q.w)
            self.amcl = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw,
                         list(msg.pose.covariance), lc.stamp_to_sec(msg.header.stamp))
            self.amcl_count += 1
            self._add_traj(self.amcl[0], self.amcl[1], yaw)

        def _on_plan(self, msg):
            # Nav2 lập lại đường mỗi giây (BT replanning) -> chỉ giữ đường đầu tiên của mỗi goal
            if self.capture_plan and self.first_plan is None and msg.poses:
                self.first_plan = [(ps.pose.position.x, ps.pose.position.y) for ps in msg.poses]

        def _feedbackCallback(self, msg):     # ghi đè của BasicNavigator: thêm ghi quỹ đạo
            super()._feedbackCallback(msg)
            fb = msg.feedback
            if hasattr(fb, 'current_pose'):
                pp = fb.current_pose.pose
                self._add_traj(pp.position.x, pp.position.y,
                               lc.yaw_from_quat(pp.orientation.x, pp.orientation.y,
                                                pp.orientation.z, pp.orientation.w))

        def _add_traj(self, x, y, yaw):
            t = self.now_sec()
            if self.trajectory:
                lt, lx, ly, _ = self.trajectory[-1]
                if math.hypot(x - lx, y - ly) < 0.05 and t - lt < 1.0:
                    return
            self.trajectory.append([round(t, 2), round(x, 3), round(y, 3), round(yaw, 3)])

        # -------------------------------------------------------------- tiện ích
        def now_sec(self):
            return self.get_clock().now().nanoseconds * 1e-9

        def param(self, name):
            return self.get_parameter(name).value

        def spin_for(self, seconds_wall):
            end = time.monotonic() + seconds_wall
            while time.monotonic() < end:
                rclpy.spin_once(self, timeout_sec=min(0.1, max(0.0, end - time.monotonic())))

        def sleep_sim(self, seconds):
            """Chờ theo giờ mô phỏng (vẫn xử lý callback)."""
            start = self.now_sec()
            wall_end = time.monotonic() + max(10.0, seconds * 20.0)   # phòng /clock đứng
            while self.now_sec() - start < seconds and time.monotonic() < wall_end:
                rclpy.spin_once(self, timeout_sec=0.05)

        def wait_until(self, pred, timeout_wall, waiting_msg=None, every=10.0):
            start = time.monotonic()
            last = start
            while not pred():
                if time.monotonic() - start > timeout_wall:
                    return False
                rclpy.spin_once(self, timeout_sec=0.1)
                if waiting_msg and time.monotonic() - last > every:
                    last = time.monotonic()
                    self.info('{} ({:.0f} s)'.format(waiting_msg, last - start))
            return True

        def lifecycle_state(self, node_name, timeout=2.0):
            cli = self.create_client(GetState, '{}/get_state'.format(node_name))
            try:
                if not cli.wait_for_service(timeout_sec=timeout):
                    return None
                fut = cli.call_async(GetState.Request())
                rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
                res = fut.result()
                return None if res is None else res.current_state.label
            finally:
                self.destroy_client(cli)

        def wait_active(self, node_name, deadline):
            last_print = 0.0
            while time.monotonic() < deadline:
                state = self.lifecycle_state(node_name)
                if state == 'active':
                    self.info('  {} : active'.format(node_name))
                    return True
                if time.monotonic() - last_print > 10.0:
                    last_print = time.monotonic()
                    self.info('  Chờ {} ... (trạng thái: {})'.format(
                        node_name, state or 'chưa có node / service get_state'))
                self.spin_for(1.0)
            return False

        def make_pose(self, x, y, yaw):
            ps = PoseStamped()
            ps.header.frame_id = 'map'
            ps.header.stamp = self.get_clock().now().to_msg()
            ps.pose.position.x, ps.pose.position.y = float(x), float(y)
            qx, qy, qz, qw = lc.quat_from_yaw(yaw)
            ps.pose.orientation.x, ps.pose.orientation.y = qx, qy
            ps.pose.orientation.z, ps.pose.orientation.w = qz, qw
            return ps

        # -------------------------------------------------------------- marker RViz
        def publish_markers(self, active=None):
            colors = {'pending': (1.0, 0.85, 0.0), 'active': (0.1, 0.5, 1.0),
                      'SUCCEEDED': (0.1, 0.8, 0.1)}
            arr = MarkerArray()
            clear = Marker()
            clear.action = Marker.DELETEALL
            arr.markers.append(clear)
            for k, g in enumerate(self.goals):
                state = self.goal_states[k] if k < len(self.goal_states) else 'pending'
                if active == k:
                    state = 'active'
                r, gg, b = colors.get(state, (0.9, 0.1, 0.1))
                for kind in ('arrow', 'text'):
                    m = Marker()
                    m.header.frame_id = 'map'
                    m.ns = 'nav_goals_' + kind
                    m.id = k
                    m.action = Marker.ADD
                    m.pose = self.make_pose(g['x'], g['y'], g['yaw']).pose
                    m.color.r, m.color.g, m.color.b, m.color.a = r, gg, b, 1.0
                    if kind == 'arrow':
                        m.type = Marker.ARROW
                        m.scale.x, m.scale.y, m.scale.z = 0.6, 0.12, 0.12
                    else:
                        m.type = Marker.TEXT_VIEW_FACING
                        m.pose.position.z = 0.5
                        m.scale.z = 0.45
                        m.text = 'G{}'.format(k + 1)
                    arr.markers.append(m)
            self.marker_pub.publish(arr)

        # -------------------------------------------------------------- các bước
        def set_initial_pose(self, x, y, yaw):
            msg = PoseWithCovarianceStamped()
            msg.header.frame_id = 'map'
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.pose.pose = self.make_pose(x, y, yaw).pose
            cov = [0.0] * 36
            cov[0] = cov[7] = INIT_COV_XY
            cov[35] = INIT_COV_YAW
            msg.pose.covariance = cov
            before = self.amcl_count
            for attempt in range(5):
                msg.header.stamp = self.get_clock().now().to_msg()
                self.init_pub.publish(msg)
                if self.wait_until(lambda: self.amcl_count > before, 3.0):
                    return True
                self.info('  Chưa thấy /amcl_pose mới – gửi lại initial pose ({}/5)'.format(attempt + 2))
            return False

        def undock(self):
            if self.undock_client is None:
                self.warn('Không có irobot_create_msgs -> bỏ qua undock')
                return True
            if not self.undock_client.wait_for_server(timeout_sec=30.0):
                self.error('Không có action server /undock (Create 3 chưa chạy?)')
                return False
            self.info('Undock ... (robot lùi ra khỏi dock rồi quay 180°)')
            fut = self.undock_client.send_goal_async(Undock.Goal())
            rclpy.spin_until_future_complete(self, fut, timeout_sec=30.0)
            handle = fut.result()
            if handle is None or not handle.accepted:
                self.error('Undock bị từ chối')
                return False
            res_fut = handle.get_result_async()
            start = self.now_sec()
            wall_end = time.monotonic() + 300.0
            while not res_fut.done():
                rclpy.spin_once(self, timeout_sec=0.1)
                if self.now_sec() - start > 60.0 or time.monotonic() > wall_end:
                    self.error('Undock quá 60 s (giờ mô phỏng) -> huỷ')
                    handle.cancel_goal_async()
                    return False
            status = res_fut.result().status
            self.wait_until(lambda: self.is_docked is False, 5.0)
            if status == GoalStatus.STATUS_SUCCEEDED and self.is_docked is not True:
                self.info('Undock thành công (is_docked = {})'.format(self.is_docked))
                return True
            self.error('Undock thất bại: status={} is_docked={} (xem README mục xử lý sự cố: '
                       'cliff_reflex / software_render)'.format(status, self.is_docked))
            return False

        def run_task(self, label, timeout, describe):
            """Chờ task hiện tại (goToPose/spin) xong. Trả về 'SUCCEEDED'/'FAILED'/'CANCELED'/'TIMEOUT'."""
            # task_active chỉ được xoá khi task KẾT THÚC bình thường; Ctrl+C (KeyboardInterrupt) giữ cờ để
            # main() biết phải huỷ goal đang chạy.
            self.task_active = True
            start = self.now_sec()
            last_print = time.monotonic()
            while not self.isTaskComplete():
                fb = self.getFeedback()
                elapsed = self.now_sec() - start
                if elapsed > timeout:
                    self.warn('{}: quá {:.0f} s (giờ mô phỏng) -> huỷ'.format(label, timeout))
                    self.cancelTask()
                    self.wait_until(self.isTaskComplete, 10.0)
                    self.task_active = False
                    return 'TIMEOUT'
                if describe and fb is not None and time.monotonic() - last_print > 3.0:
                    last_print = time.monotonic()
                    self.info(describe(fb, elapsed))
            self.task_active = False
            result = self.getResult()
            return {TaskResult.SUCCEEDED: 'SUCCEEDED', TaskResult.FAILED: 'FAILED',
                    TaskResult.CANCELED: 'CANCELED'}.get(result, 'UNKNOWN')

        def navigate(self, k, goal, label):
            pose = self.make_pose(goal['x'], goal['y'], goal['yaw'])
            timeout = float(self.param('goal_timeout'))
            retries = int(self.param('max_retries'))
            t0 = self.now_sec()
            traj_start = len(self.trajectory)
            plan = None
            status = 'UNKNOWN'
            attempts = 0
            for attempt in range(retries + 1):
                attempts = attempt + 1
                if attempt:
                    self.warn('{}: thử lại lần {} (xoá costmap trước)'.format(label, attempt))
                    try:
                        self.clearAllCostmaps()
                    except Exception as exc:  # noqa: BLE001
                        self.warn('Không xoá được costmap: {}'.format(exc))
                self.feedback = None
                if plan is None:
                    self.first_plan, self.capture_plan = None, True
                if not self.goToPose(pose):
                    status = 'REJECTED'
                    continue

                def describe(fb, elapsed):
                    eta = Duration.from_msg(fb.estimated_time_remaining).nanoseconds * 1e-9
                    return '  -> {}: còn {:.2f} m | ETA ~{:.0f} s | đã đi {:.0f} s | recovery {}'.format(
                        label, fb.distance_remaining, eta, elapsed, fb.number_of_recoveries)
                status = self.run_task(label, timeout, describe)
                if plan is None and self.first_plan:
                    plan = self.first_plan
                self.capture_plan = False
                if status == 'SUCCEEDED':
                    break
                if status in ('CANCELED',):
                    break
            fb = self.getFeedback()
            recov = fb.number_of_recoveries if fb is not None else 0
            err_xy = err_yaw = None
            last = None
            if fb is not None and hasattr(fb, 'current_pose'):     # pose cuối cùng bt_navigator báo
                pp = fb.current_pose.pose
                last = (pp.position.x, pp.position.y,
                        lc.yaw_from_quat(pp.orientation.x, pp.orientation.y, pp.orientation.z,
                                         pp.orientation.w))
            elif self.trajectory:
                last = tuple(self.trajectory[-1][1:])
            if last is not None:
                x, y, yaw = last
                err_xy = math.hypot(x - goal['x'], y - goal['y'])
                err_yaw = abs(lc.normalize_angle(yaw - goal['yaw']))
            pts = [(p[1], p[2]) for p in self.trajectory[traj_start:]]
            return {'label': label, 'name': goal['name'], 'goal': [goal['x'], goal['y'], goal['yaw']],
                    'status': status, 'duration': round(self.now_sec() - t0, 2),
                    'distance': round(path_length(pts), 2), 'error_xy': err_xy, 'error_yaw': err_yaw,
                    'recoveries': recov, 'attempts': attempts,
                    'plan': None if plan is None else [list(p) for p in plan[::3] + plan[-1:]]}

        def look_around(self, label):
            self.info('{}: xoay 360° để camera quét xung quanh'.format(label))
            if not self.spin(spin_dist=2.0 * math.pi, time_allowance=40):
                return 'REJECTED'
            return self.run_task(label + ' (xoay)', 60.0, None)

        def dock(self):
            if self.dock_client is None or not self.dock_client.wait_for_server(timeout_sec=10.0):
                self.warn('Không có action server /dock -> bỏ qua')
                return False
            fut = self.dock_client.send_goal_async(Dock.Goal())
            rclpy.spin_until_future_complete(self, fut, timeout_sec=30.0)
            handle = fut.result()
            if handle is None or not handle.accepted:
                return False
            res_fut = handle.get_result_async()
            start = self.now_sec()
            while not res_fut.done() and self.now_sec() - start < 120.0:
                rclpy.spin_once(self, timeout_sec=0.1)
            return res_fut.done() and res_fut.result().status == GoalStatus.STATUS_SUCCEEDED

    return TB4WaypointNavigator


# ------------------------------------------------------------------ chương trình chính
def save_results(out_dir, nav, init, results, goals_path):
    os.makedirs(out_dir, exist_ok=True)
    run = {'goals_file': goals_path, 'initial_pose': list(init),
           'goals': nav.goals, 'results': results,
           'trajectory': nav.trajectory,
           'plans': [r.get('plan') for r in results if not r.get('extra')]}
    with open(os.path.join(out_dir, 'run.json'), 'w', encoding='utf-8') as f:
        json.dump(run, f, ensure_ascii=False, indent=1)
    with open(os.path.join(out_dir, 'summary.csv'), 'w', newline='', encoding='utf-8') as f:
        w = csv.writer(f)
        w.writerow(['goal', 'name', 'x', 'y', 'yaw', 'status', 'duration_s', 'distance_m',
                    'error_xy_m', 'error_yaw_rad', 'recoveries', 'attempts'])
        for r in results:
            w.writerow([r['label'], r['name'], *r['goal'], r['status'], r['duration'], r['distance'],
                        '' if r['error_xy'] is None else round(r['error_xy'], 3),
                        '' if r['error_yaw'] is None else round(r['error_yaw'], 3),
                        r['recoveries'], r['attempts']])


def main():
    import signal
    import rclpy
    # Mặc định rclpy (Humble) ĐÓNG context ngay khi nhận SIGINT/SIGTERM -> khối finally không còn gửi được
    # lệnh huỷ goal và robot cứ đi tiếp tới goal. Vì vậy tắt bộ xử lý tín
    # hiệu của rclpy; Ctrl+C / SIGTERM (ros2 launch tắt) chỉ ném KeyboardInterrupt, ta huỷ goal rồi mới shutdown.
    try:
        from rclpy.signals import SignalHandlerOptions
        rclpy.init(args=sys.argv, signal_handler_options=SignalHandlerOptions.NO)
    except (ImportError, TypeError):
        rclpy.init(args=sys.argv)
    signal.signal(signal.SIGTERM, lambda signum, frame: (_ for _ in ()).throw(KeyboardInterrupt()))
    NavClass = build_node_class()
    nav = NavClass()
    results = []
    init = (0.0, 0.0, 0.0)
    goals_path = None
    out_dir = None
    code = 1
    try:
        goals_path = nav.param('goals_file') or lc.find_config('config', 'goals.yaml')[0]
        if not goals_path or not os.path.isfile(goals_path):
            nav.error('Không tìm thấy file goal: {}'.format(goals_path or 'config/goals.yaml'))
            return 2
        try:
            init, goals = load_goals(goals_path)
        except (OSError, ValueError) as exc:
            nav.error('File goal lỗi: {}'.format(exc))
            return 2
        nav.goals = goals
        nav.goal_states = ['pending'] * len(goals)
        out_dir = nav.param('results_dir') or os.path.join(lc.results_dir(), 'nav_' + lc.timestamp())
        nav.info('Đọc {} goal từ {}:'.format(len(goals), goals_path))
        for k, g in enumerate(goals):
            nav.info('  G{}: x={:.2f} y={:.2f} yaw={:.2f} rad ({:.0f}°)  {}'.format(
                k + 1, g['x'], g['y'], g['yaw'], math.degrees(g['yaw']), g['name']))
        nav.publish_markers()

        # ---- 1. Nav2 + AMCL sẵn sàng
        nav.info('[1/5] Chờ AMCL và Nav2 (bt_navigator) ở trạng thái active ...')
        deadline = time.monotonic() + float(nav.param('startup_timeout'))
        if not nav.wait_until(lambda: nav.now_sec() > 0.0, 60.0, 'Chờ /clock (giờ mô phỏng)'):
            nav.error('Không nhận được /clock – mô phỏng chưa chạy?')
            return 3
        if not nav.wait_active('amcl', deadline):
            nav.error('AMCL không active – đã chạy `ros2 launch tb4_lab04 navigation.launch.py` chưa?')
            return 3
        # dock_status (nếu có) để quyết định initial pose
        nav.wait_until(lambda: nav.is_docked is not None, 15.0)

        # ---- 2. initial pose
        nav.info('[2/5] Initial pose cho AMCL')
        do_set, why = decide_initial_pose(nav.param('initial_pose_mode'), nav.is_docked)
        if do_set:
            nav.info('  Đặt initial pose ({:.2f}, {:.2f}, yaw {:.2f}) – {}'.format(*init, why))
            if not nav.set_initial_pose(*init):
                nav.warn('  AMCL chưa phản hồi /amcl_pose sau khi đặt initial pose – tiếp tục, '
                         'kiểm tra trên RViz2 (chấm LiDAR phải trùng tường)')
        else:
            nav.info('  Không đặt lại: {}'.format(why))
        nav.wait_until(lambda: nav.amcl is not None, 10.0)
        if nav.amcl is not None:
            sx, sy, syaw = pose_std(nav.amcl[3])
            nav.info('  AMCL: ({:.2f}, {:.2f}, {:.0f}°)  độ lệch chuẩn x={:.2f} m y={:.2f} m '
                     'yaw={:.0f}°'.format(nav.amcl[0], nav.amcl[1], math.degrees(nav.amcl[2]),
                                         sx, sy, math.degrees(syaw)))
            if max(sx, sy) > 0.6:
                nav.warn('  AMCL còn rất không chắc chắn – sẽ hội tụ khi robot di chuyển; nếu robot '
                         'đi sai hướng ngay từ đầu: Ctrl+C và đặt 2D Pose Estimate trên RViz2')

        if not nav.wait_active('bt_navigator', deadline):
            nav.error('bt_navigator không active – Nav2 chưa lên (xem log Terminal launch)')
            return 3
        nav.info('Nav2 sẵn sàng.')

        # ---- 3. undock
        nav.info('[3/5] Rời dock')
        if nav.param('undock') and nav.is_docked:
            if not nav.undock():
                return 4
        elif nav.is_docked is None:
            nav.warn('  Không có /dock_status -> bỏ qua undock')
        else:
            nav.info('  Robot không ở trong dock -> bỏ qua undock')
        nav.sleep_sim(1.0)
        home = None
        if nav.trajectory:
            _, hx, hy, hyaw = nav.trajectory[-1]
            home = (hx, hy, hyaw)

        # ---- 4. goal
        loops = max(1, int(nav.param('loops')))
        nav.info('[4/5] Bắt đầu đi qua {} goal{} – KHÔNG điều khiển tay từ lúc này'.format(
            len(goals), '' if loops == 1 else ' x {} vòng'.format(loops)))
        for loop in range(loops):
            nav.goal_states = ['pending'] * len(goals)
            for k, g in enumerate(goals):
                label = 'G{}'.format(k + 1) if loops == 1 else 'V{}-G{}'.format(loop + 1, k + 1)
                nav.info('=== {} / {}: {} -> ({:.2f}, {:.2f}, {:.0f}°)'.format(
                    label, len(goals), g['name'], g['x'], g['y'], math.degrees(g['yaw'])))
                nav.publish_markers(active=k)
                r = nav.navigate(k, g, label)
                nav.goal_states[k] = r['status']
                nav.publish_markers()
                results.append(r)
                if r['status'] == 'SUCCEEDED':
                    nav.info('<<< {}: TỚI NƠI sau {:.1f} s, đi {:.1f} m, sai số {:.2f} m / {:.0f}°'.format(
                        label, r['duration'], r['distance'], r['error_xy'] or 0.0,
                        math.degrees(r['error_yaw'] or 0.0)))
                    if nav.param('look_around'):
                        nav.look_around(label)
                    pause = float(nav.param('pause_at_goal'))
                    if pause > 0:
                        nav.info('    dừng {:.0f} s tại {} (YOLO quan sát)'.format(pause, label))
                        nav.sleep_sim(pause)
                else:
                    nav.error('<<< {}: {} sau {:.1f} s – chuyển sang goal tiếp theo'.format(
                        label, STATUS_TEXT.get(r['status'], r['status']), r['duration']))

        if nav.param('return_home') and home is not None:
            nav.info('=== Về chỗ sau khi undock ({:.2f}, {:.2f})'.format(home[0], home[1]))
            back_yaw = lc.normalize_angle(home[2] + math.pi)     # quay mặt về phía dock
            r = nav.navigate(len(goals), {'name': 'home', 'x': home[0], 'y': home[1], 'yaw': back_yaw},
                             'HOME')
            r['extra'] = True
            results.append(r)
            if r['status'] == 'SUCCEEDED' and nav.param('dock_at_end'):
                nav.info('Vào dock ...')
                nav.info('Dock {}'.format('thành công' if nav.dock() else 'KHÔNG thành công'))

        # ---- 5. tổng kết
        lines, ok, total = summary_lines(results)
        nav.info('[5/5] TỔNG KẾT')
        for line in lines:
            print(line, flush=True)
        code = 0 if ok == total and total > 0 else 1
        print('=> {}'.format('ĐẠT: robot tự đi qua {} goal liên tiếp'.format(total) if code == 0
                             else 'CHƯA ĐẠT: có goal thất bại (xem log ở trên)'), flush=True)
        return code
    finally:
        # Từ đây bỏ qua Ctrl+C lần 2 / SIGTERM của ros2 launch: để lệnh huỷ goal tới được bt_navigator
        # và kết quả được lưu trọn vẹn.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        if nav.task_active and rclpy.ok():
            try:
                print('[tb4_lab04] Huỷ goal đang chạy để robot dừng ...', flush=True)
                fut = nav.goal_handle.cancel_goal_async()
                rclpy.spin_until_future_complete(nav, fut, timeout_sec=5.0)
                print('[tb4_lab04] Đã gửi lệnh huỷ – robot dừng.' if fut.done() else
                      '[tb4_lab04] Không xác nhận được lệnh huỷ (bt_navigator không phản hồi).', flush=True)
            except Exception as exc:  # noqa: BLE001
                print('[tb4_lab04] Huỷ goal lỗi: {}'.format(exc), flush=True)
        if out_dir and results:
            try:
                save_results(out_dir, nav, init, results, goals_path)
                print('[tb4_lab04] Đã lưu kết quả: {}/run.json, summary.csv'.format(out_dir), flush=True)
                print('            Vẽ ảnh: ros2 run tb4_lab04 check_goals.py --plot {0}/map_run.png '
                      '--run {0}/run.json'.format(out_dir), flush=True)
            except OSError as exc:
                print('[tb4_lab04] Không lưu được kết quả: {}'.format(exc), flush=True)
        try:
            nav.destroy_node()
        except Exception:  # noqa: BLE001
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    lc.install_ctrl_c_handlers()
    # Ctrl+C là thao tác chủ động của người dùng (goal đã được huỷ) -> mã 0, ros2 launch không báo lỗi đỏ
    sys.exit(lc.run_main(main, interrupted_code=0))
