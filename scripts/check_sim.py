#!/usr/bin/env python3
# Bài thực hành 04 – Phần 1: kiểm tra tự động mô phỏng TurtleBot 4.
#
# Chạy SAU khi `ros2 launch tb4_lab04 sim.launch.py` đã lên (robot đã xuất hiện):
#     ros2 run tb4_lab04 check_sim.py              # kiểm tra topic + lái thử robot
#     ros2 run tb4_lab04 check_sim.py --no-drive   # chỉ kiểm tra topic, không lái
#
# Kiểm tra:
#   1. /scan (sensor_msgs/LaserScan) có dữ liệu, tần số, số tia hợp lệ
#   2. /odom (nav_msgs/Odometry) có dữ liệu, frame odom -> base_link
#   3. TF odom -> base_link và base_link -> frame của LiDAR
#   4. /cmd_vel có node nhận (motion_control của Create 3)
#   5. (mặc định) Undock nếu robot đang nằm trong dock, rồi gửi /cmd_vel:
#      tiến thẳng một đoạn ngắn (chỉ khi phía trước trống) và quay tại chỗ,
#      đo thay đổi trên /odom để chứng minh robot nhận lệnh vận tốc.
#
# Lưu ý: TurtleBot 4 mô phỏng KHỞI ĐỘNG TRONG DOCK, mặt hướng vào dock. Lái tiến khi
# còn trong dock sẽ đâm vào dock -> script luôn undock trước (trừ khi --no-undock).

import argparse
import math
import os
import signal
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from check_sensors import HAZARD_NAMES, REFLEX_HAZARDS, RENDER_FIX, SCAN_ADVICE, scan_health  # noqa: E402

# ---------------------------------------------------------------------------
# Hàm thuần (không cần ROS) – có unit test trong test/test_pure_logic.py
# ---------------------------------------------------------------------------


def yaw_from_quaternion(x, y, z, w):
    """Góc yaw (rad) từ quaternion."""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def wrap_angle(a):
    """Đưa góc về (-pi, pi]."""
    a = math.fmod(a + math.pi, 2.0 * math.pi)
    if a <= 0.0:
        a += 2.0 * math.pi
    return a - math.pi


def count_valid_ranges(ranges, range_min, range_max):
    """Số tia LiDAR hữu hạn và nằm trong [range_min, range_max]."""
    n = 0
    for r in ranges:
        if math.isfinite(r) and range_min <= r <= range_max:
            n += 1
    return n


def scan_points_in_base(ranges, angle_min, angle_increment, range_min, range_max,
                        tx, ty, tyaw):
    """Chuyển các tia hợp lệ sang toạ độ base_link (2D).

    (tx, ty, tyaw) = pose của LiDAR trong frame base_link.
    """
    c, s = math.cos(tyaw), math.sin(tyaw)
    pts = []
    for i, r in enumerate(ranges):
        if not (math.isfinite(r) and range_min <= r <= range_max):
            continue
        a = angle_min + i * angle_increment
        lx, ly = r * math.cos(a), r * math.sin(a)
        pts.append((tx + c * lx - s * ly, ty + s * lx + c * ly))
    return pts


def path_is_clear(points, distance, half_width):
    """True nếu không có điểm nào trong vùng phía trước robot.

    Vùng: 0 < x <= distance và |y| <= half_width (frame base_link).
    """
    for x, y in points:
        if 0.0 < x <= distance and abs(y) <= half_width:
            return False
    return True


class StreamStats:
    """Đếm message và tính tần số theo thời gian thực (wall) và theo header.stamp (sim)."""

    def __init__(self):
        self.count = 0
        self.first_wall = None
        self.last_wall = None
        self.first_stamp = None
        self.last_stamp = None

    def add(self, wall_t, stamp_t):
        self.count += 1
        if self.first_wall is None:
            self.first_wall = wall_t
            self.first_stamp = stamp_t
        self.last_wall = wall_t
        self.last_stamp = stamp_t

    def wall_rate(self):
        if self.count < 2 or self.last_wall <= self.first_wall:
            return 0.0
        return (self.count - 1) / (self.last_wall - self.first_wall)

    def stamp_rate(self):
        if self.count < 2 or self.last_stamp is None or self.last_stamp <= self.first_stamp:
            return 0.0
        return (self.count - 1) / (self.last_stamp - self.first_stamp)


# ---------------------------------------------------------------------------
# Phần ROS
# ---------------------------------------------------------------------------


def main(argv=None):
    parser = argparse.ArgumentParser(description='Kiểm tra mô phỏng TurtleBot 4 (Phần 1).')
    parser.add_argument('--no-drive', action='store_true', help='không gửi /cmd_vel')
    parser.add_argument('--no-undock', action='store_true', help='không gọi action /undock')
    parser.add_argument('--window', type=float, default=5.0,
                        help='số giây (wall) đo tần số topic (mặc định 5)')
    parser.add_argument('--timeout', type=float, default=60.0,
                        help='số giây tối đa chờ topic xuất hiện (mặc định 60)')
    # rclpy tự lọc --ros-args; argparse chỉ nhận phần còn lại.
    import rclpy
    from rclpy.utilities import remove_ros_args
    args = parser.parse_args(remove_ros_args(argv if argv is not None else sys.argv)[1:])

    from rclpy.action import ActionClient
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.time import Time
    from geometry_msgs.msg import Twist
    from irobot_create_msgs.action import Undock
    from irobot_create_msgs.msg import DockStatus, HazardDetection, HazardDetectionVector
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import LaserScan
    from tf2_ros import TransformException
    from tf2_ros.buffer import Buffer
    from tf2_ros.transform_listener import TransformListener

    rclpy.init(args=argv)
    node = Node('tb4_lab04_check_sim',
                parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])
    log = node.get_logger()

    results = []  # (tên, ok, chi tiết)

    def record(name, ok, detail):
        results.append((name, ok, detail))
        # rclpy cấm đổi mức log tại CÙNG một dòng lệnh giữa các lần gọi
        # ("Logger severity cannot be changed between calls") -> tách 2 nhánh.
        if ok:
            log.info('[PASS] {}: {}'.format(name, detail))
        else:
            log.error('[FAIL] {}: {}'.format(name, detail))

    scan_stats, odom_stats = StreamStats(), StreamStats()
    state = {'scan': None, 'odom': None, 'dock': None, 'bumps': 0, 'reflex_hz': set()}

    def stamp_sec(h):
        return h.stamp.sec + h.stamp.nanosec * 1e-9

    def on_scan(msg):
        state['scan'] = msg
        scan_stats.add(time.monotonic(), stamp_sec(msg.header))

    def on_odom(msg):
        state['odom'] = msg
        odom_stats.add(time.monotonic(), stamp_sec(msg.header))

    def on_dock(msg):
        state['dock'] = msg

    def on_hazard(msg):
        for d in msg.detections:
            if d.type == HazardDetection.BUMP:
                state['bumps'] += 1
            name = HAZARD_NAMES.get(d.type, str(d.type))
            if name in REFLEX_HAZARDS:
                state['reflex_hz'].add(name)

    node.create_subscription(LaserScan, '/scan', on_scan, qos_profile_sensor_data)
    node.create_subscription(Odometry, '/odom', on_odom, qos_profile_sensor_data)
    node.create_subscription(DockStatus, '/dock_status', on_dock, qos_profile_sensor_data)
    node.create_subscription(HazardDetectionVector, '/hazard_detection', on_hazard,
                             qos_profile_sensor_data)
    cmd_pub = node.create_publisher(Twist, '/cmd_vel', 10)
    tf_buffer = Buffer()
    tf_listener = TransformListener(tf_buffer, node)  # noqa: F841 – phải giữ tham chiếu

    def spin_for(seconds):
        end = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.05)

    def spin_until(pred, timeout):
        end = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < end:
            if pred():
                return True
            rclpy.spin_once(node, timeout_sec=0.05)
        return pred()

    def lookup_2d(target, source):
        try:
            t = tf_buffer.lookup_transform(target, source, Time())
        except TransformException:
            return None
        q = t.transform.rotation
        return (t.transform.translation.x, t.transform.translation.y,
                yaw_from_quaternion(q.x, q.y, q.z, q.w))

    def sim_now():
        return node.get_clock().now().nanoseconds * 1e-9

    def stop_robot():
        for _ in range(5):
            cmd_pub.publish(Twist())
            spin_for(0.05)

    exit_code = 1
    try:
        # ---------------- 1-2. Topic /scan, /odom ----------------
        log.info('Đang chờ /scan và /odom (tối đa {:.0f} s)...'.format(args.timeout))
        spin_until(lambda: state['scan'] is not None and state['odom'] is not None, args.timeout)
        log.info('Đo tần số trong {:.1f} s...'.format(args.window))
        sim_t0, wall_t0 = sim_now(), time.monotonic()
        spin_for(args.window)
        sim_dt, wall_dt = sim_now() - sim_t0, time.monotonic() - wall_t0
        rtf = sim_dt / wall_dt if wall_dt > 0 else 0.0

        scan = state['scan']
        if scan is None:
            record('/scan', False, 'không nhận được message nào – kiểm tra Gazebo/bridge')
        else:
            n_valid = count_valid_ranges(scan.ranges, scan.range_min, scan.range_max)
            # Tần số theo header.stamp (giờ mô phỏng) – không phụ thuộc máy nhanh/chậm
            ok = n_valid > 0 and scan_stats.stamp_rate() >= 5.0
            record('/scan', ok,
                   'frame={} | {} tia ({} hợp lệ) | {:.1f} Hz (wall), {:.1f} Hz (sim) | '
                   'range [{:.2f}, {:.2f}] m'.format(
                       scan.header.frame_id, len(scan.ranges), n_valid, scan_stats.wall_rate(),
                       scan_stats.stamp_rate(), scan.range_min, scan.range_max))
            # "Có dữ liệu" chưa đủ: LiDAR trong Gazebo Fortress là gpu_lidar (đo bằng render đồ hoạ);
            # driver đồ hoạ lỗi -> topic vẫn chạy đều nhưng số đo vô nghĩa.
            h = scan_health(scan.ranges, scan.range_min, scan.range_max)
            record('/scan: số đo hợp lý', h['verdict'] == 'ok',
                   'min/trung vị/max = {:.2f}/{:.2f}/{:.2f} m, {:.0f} % tia sát range_min{}'.format(
                       h['min'], h['median'], h['max'], 100 * h['frac_near_min'],
                       '' if h['verdict'] == 'ok' else ' -> ' + SCAN_ADVICE[h['verdict']] +
                       ' Chạy: ros2 run tb4_lab04 check_sensors.py'))

        odom = state['odom']
        if odom is None:
            record('/odom', False, 'không nhận được message nào – diffdrive_controller chưa chạy?')
        else:
            ok = odom.header.frame_id.endswith('odom') and odom_stats.stamp_rate() >= 5.0
            record('/odom', ok, 'frame={} -> {} | {:.1f} Hz (wall), {:.1f} Hz (sim)'.format(
                odom.header.frame_id, odom.child_frame_id, odom_stats.wall_rate(),
                odom_stats.stamp_rate()))

        if rtf > 0:
            log.info('Real-time factor ước lượng ≈ {:.2f} (1.0 = thời gian thực){}'.format(
                rtf, '  <- máy đang chậm, SLAM/Nav2 sẽ chậm theo' if rtf < 0.5 else ''))
        else:
            log.warn('Không đo được sim time – /clock có đang được publish không?')

        # ---------------- 3. TF ----------------
        odom_base = lookup_2d('odom', 'base_link')
        record('TF odom->base_link', odom_base is not None,
               'OK' if odom_base else 'không có transform')
        lidar = None
        if scan is not None:
            lidar = lookup_2d('base_link', scan.header.frame_id)
            record('TF base_link->' + scan.header.frame_id, lidar is not None,
                   'x={:.3f} y={:.3f} yaw={:.2f} rad'.format(*lidar) if lidar
                   else 'không có transform (RViz sẽ không vẽ được scan)')

        # ---------------- 4. /cmd_vel ----------------
        n_sub = node.count_subscribers('/cmd_vel')
        record('/cmd_vel có subscriber', n_sub > 0, '{} subscriber'.format(n_sub))

        # ---------------- 5. Lái thử ----------------
        if not args.no_drive and odom is not None and n_sub > 0:
            # /dock_status chạy theo giờ mô phỏng (20 Hz sim): máy chậm (RTF thấp) có thể vài
            # giây thực mới có 1 message -> chờ đủ lâu. KHÔNG biết trạng thái dock thì KHÔNG lái
            # (nếu robot còn trong dock mà lái tiến sẽ đâm vào dock).
            dock_wait = min(120.0, max(30.0, 3.0 / max(rtf, 0.01)))
            if not spin_until(lambda: state['dock'] is not None, dock_wait):
                record('/dock_status', False,
                       'không nhận được trong {:.0f} s – không xác định được robot có trong '
                       'dock hay không nên bỏ qua lái thử'.format(dock_wait))
                docked = None
            else:
                # Cập nhật bằng KẾT QUẢ action undock bên dưới (không chờ /dock_status lần sau
                # để tránh đọc giá trị cũ).
                docked = state['dock'].is_docked
            undock_bad = False      # undock bị cắt ngang -> robot vẫn quay mặt vào dock
            if docked and not args.no_undock:
                log.info('Robot đang trong dock -> gửi action /undock ...')
                client = ActionClient(node, Undock, '/undock')
                if not client.wait_for_server(timeout_sec=20.0):
                    record('Undock', False, 'không thấy action server /undock')
                else:
                    gh_future = client.send_goal_async(Undock.Goal())
                    rclpy.spin_until_future_complete(node, gh_future, timeout_sec=20.0)
                    gh = gh_future.result()
                    if gh is None or not gh.accepted:
                        record('Undock', False, 'goal bị từ chối hoặc không phản hồi')
                    else:
                        res_future = gh.get_result_async()
                        # Chờ theo GIỜ MÔ PHỎNG (tối đa 60 s sim): máy chậm có RTF rất thấp,
                        # chờ theo giờ thực sẽ hết hạn trước khi robot kịp undock.
                        start_sim = sim_now()
                        wall_end = time.monotonic() + max(120.0, 60.0 / max(rtf, 0.01))
                        while rclpy.ok() and not res_future.done() \
                                and sim_now() - start_sim < 60.0 and time.monotonic() < wall_end:
                            rclpy.spin_once(node, timeout_sec=0.1)
                        res = res_future.result() if res_future.done() else None
                        # Undock đúng = lùi ~0.4 m rồi quay 180 độ (mặt quay khỏi dock). Kết quả
                        # CANCELED + is_docked=False nghĩa là bị PHẢN XẠ (cliff/bumper) cắt ngang:
                        # robot chỉ nhích vài cm, vẫn đứng sát và quay mặt vào dock -> KHÔNG đạt.
                        status = {4: 'SUCCEEDED', 5: 'CANCELED', 6: 'ABORTED'}.get(
                            res.status, str(res.status)) if res is not None else 'không có kết quả'
                        if state['odom'] is not None and odom is not None:
                            p0, p1 = odom.pose.pose.position, state['odom'].pose.pose.position
                            moved = math.hypot(p1.x - p0.x, p1.y - p0.y)
                        else:
                            moved = float('nan')
                        left = res is not None and not res.result.is_docked
                        ok = left and status == 'SUCCEEDED' and moved > 0.25
                        detail = 'status={}, is_docked={}, robot đã lùi {:.2f} m (kỳ vọng ~0.4 m)'.format(
                            status, None if res is None else res.result.is_docked, moved)
                        if state['reflex_hz']:
                            detail += ', hazard gây phản xạ: ' + ', '.join(sorted(state['reflex_hz']))
                        if not ok:
                            detail += (' -> undock bị cắt ngang, robot vẫn sát dock. Chạy '
                                       'check_sensors.py để xem cảm biến cliff/LiDAR')
                        record('Undock', ok, detail)
                        undock_bad = not ok
                        if left:
                            docked = False
                spin_for(1.0)

            if docked is None:
                pass
            elif docked:
                if args.no_undock:
                    log.warn('Robot đang trong dock và bạn chọn --no-undock: bỏ qua phần lái thử.')
                else:
                    record('cmd_vel: lái thử', False,
                           'robot vẫn trong dock nên không lái thử (lái khi còn trong dock sẽ '
                           'đâm vào dock)')
            else:
                bumps_before = state['bumps']

                def drive(lin, ang, sim_seconds):
                    """Gửi Twist 10 Hz tới khi đủ sim_seconds (có giới hạn theo giờ thực)."""
                    tw = Twist()
                    tw.linear.x, tw.angular.z = lin, ang
                    start_sim = sim_now()
                    deadline = time.monotonic() + max(8.0, 4.0 * sim_seconds / max(rtf, 0.05))
                    while rclpy.ok() and sim_now() - start_sim < sim_seconds \
                            and time.monotonic() < deadline:
                        cmd_pub.publish(tw)
                        spin_for(0.1)
                    stop_robot()
                    spin_for(0.5)

                def odom_pose():
                    o = state['odom'].pose.pose
                    return (o.position.x, o.position.y, yaw_from_quaternion(
                        o.orientation.x, o.orientation.y, o.orientation.z, o.orientation.w))

                # 5a. Tiến thẳng 0.15 m/s trong 2 s (~0.3 m) – chỉ khi phía trước trống 0.8 m
                scan = state['scan']
                if undock_bad:
                    # LiDAR quét CAO hơn dock nên "thấy" phía trước trống, nhưng robot đang quay mặt
                    # vào dock -> lái tiến sẽ đâm vào dock. Bỏ qua bước này.
                    log.warn('Undock không trọn vẹn (robot có thể đang quay mặt vào dock) -> bỏ qua '
                             'bước lái tiến.')
                elif lidar is None:
                    log.warn('Không có TF LiDAR -> không kiểm tra được phía trước, '
                             'bỏ qua bước lái tiến.')
                else:
                    pts = scan_points_in_base(scan.ranges, scan.angle_min, scan.angle_increment,
                                              scan.range_min, scan.range_max, *lidar)
                    if not path_is_clear(pts, 0.8, 0.25):
                        log.warn('Có vật cản trong 0.8 m phía trước -> bỏ qua bước lái tiến.')
                    else:
                        x0, y0, _ = odom_pose()
                        drive(0.15, 0.0, 2.0)
                        x1, y1, _ = odom_pose()
                        dist = math.hypot(x1 - x0, y1 - y0)
                        record('cmd_vel: tiến thẳng', dist > 0.1,
                               'quãng đường đo từ /odom = {:.3f} m (kỳ vọng ~0.3 m)'.format(dist))

                # 5b. Quay tại chỗ 0.6 rad/s trong 3 s (~1.8 rad)
                # Ngay sau undock, Create 3 có thể còn chạy một phản xạ (reflex) 8-12 s và BỎ QUA
                # /cmd_vel ("Ignoring velocities commanded while an autonomous behavior is
                # running"). Vì vậy: gửi lệnh quay liên tục, chờ tối đa 40 s mô phỏng cho tới khi
                # robot bắt đầu quay, rồi mới đo 3 s quay.
                _, _, yaw_prev = odom_pose()
                total, start_sim = 0.0, sim_now()
                move_sim = None            # giờ sim lúc robot bắt đầu quay
                warned_wait = False
                tw = Twist()
                tw.angular.z = 0.6
                wall_deadline = time.monotonic() + min(240.0, max(60.0, 45.0 / max(rtf, 0.05)))
                while rclpy.ok() and time.monotonic() < wall_deadline:
                    now = sim_now()
                    if move_sim is None and now - start_sim >= 40.0:
                        break
                    if move_sim is not None and now - move_sim >= 3.0:
                        break
                    if move_sim is None and not warned_wait and now - start_sim >= 4.0:
                        log.info('Robot chưa nhận lệnh quay (có thể đang chạy phản xạ sau undock) '
                                 '-> tiếp tục chờ tối đa 40 s mô phỏng ...')
                        warned_wait = True
                    cmd_pub.publish(tw)
                    spin_for(0.1)
                    _, _, yaw_now = odom_pose()
                    total += wrap_angle(yaw_now - yaw_prev)
                    yaw_prev = yaw_now
                    if move_sim is None and abs(total) > 0.05:
                        move_sim = now
                stop_robot()
                waited = (move_sim - start_sim) if move_sim is not None else (sim_now() - start_sim)
                detail = 'góc quay đo từ /odom = {:.2f} rad (kỳ vọng ~1.8 rad)'.format(total)
                if move_sim is not None and waited > 2.0:
                    detail += '; robot chỉ bắt đầu quay sau {:.0f} s mô phỏng (phản xạ sau undock)'.format(
                        waited)
                if move_sim is None:
                    detail += ('; robot không phản hồi lệnh trong {:.0f} s mô phỏng – kiểm tra log '
                               'motion_control ("Ignoring velocities...") và /hazard_detection'.format(
                                   waited))
                record('cmd_vel: quay tại chỗ', abs(total) > 0.5, detail)
                if state['bumps'] > bumps_before:
                    log.warn('Có {} sự kiện va chạm (bumper) trong lúc lái thử.'.format(
                        state['bumps'] - bumps_before))
        elif not args.no_drive:
            log.warn('Bỏ qua lái thử vì thiếu /odom hoặc không có subscriber /cmd_vel.')

        # ---------------- Tổng kết ----------------
        n_fail = sum(1 for _, ok, _ in results if not ok)
        print('\n=================== KẾT QUẢ KIỂM TRA ===================')
        for name, ok, detail in results:
            print('  [{}] {:<26} {}'.format('PASS' if ok else 'FAIL', name, detail))
        print('=========================================================')
        print('TẤT CẢ ĐỀU ĐẠT.' if n_fail == 0 else '{} mục KHÔNG ĐẠT – xem README mục '
              '8 "Xử lý sự cố".'.format(n_fail))
        if any(not ok and name in ('/scan: số đo hợp lý', 'Undock') for name, ok, _ in results):
            print(RENDER_FIX)
        exit_code = 0 if n_fail == 0 else 1
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            stop_robot()
        except Exception:  # noqa: BLE001 – đang tắt, bỏ qua mọi lỗi
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return exit_code


_SIGINT = {'got': False}


def _on_sigint(signum, frame):
    """Như mặc định (KeyboardInterrupt) nhưng ghi nhớ là người dùng đã bấm Ctrl+C."""
    _SIGINT['got'] = True
    raise KeyboardInterrupt


signal.signal(signal.SIGINT, _on_sigint)


def _quiet_after_ctrl_c():
    """Ctrl+C rơi đúng lúc rclpy đang khởi tạo (Executor, guard condition, import) để lại object dở dang;
    Python in "Exception ignored in ... __del__" / traceback KeyboardInterrupt ở luồng phụ. Đó là rác lúc
    tắt, không phải lỗi -> chỉ nuốt các thông báo này SAU khi người dùng đã bấm Ctrl+C."""
    import sys as _sys
    import threading as _threading
    old_unraisable, old_except, old_thread = _sys.unraisablehook, _sys.excepthook, _threading.excepthook

    def unraisable(u):
        if not _SIGINT['got']:
            old_unraisable(u)

    def except_hook(tp, val, tb):
        if issubclass(tp, KeyboardInterrupt) or (_SIGINT['got'] and issubclass(tp, Exception)):
            print('[tb4_lab04] Đã dừng (Ctrl+C).', flush=True)
            return
        old_except(tp, val, tb)

    def thread_hook(args):
        if issubclass(args.exc_type, KeyboardInterrupt) or _SIGINT['got']:
            return
        old_thread(args)

    _sys.unraisablehook, _sys.excepthook, _threading.excepthook = unraisable, except_hook, thread_hook


_quiet_after_ctrl_c()


def _ros_shutting_down():
    """True nếu đang tắt vì Ctrl+C (hoặc rclpy đã bị tắt): lỗi phát sinh lúc đó là race khi tắt của rclpy
    (vd. RuntimeError "Unable to convert call argument to Python object" khi SIGINT cắt ngang lúc nhận
    message), không phải lỗi thật. Lỗi khi chưa bấm Ctrl+C vẫn được báo bình thường."""
    if _SIGINT['got']:
        return True
    try:
        from rclpy.utilities import get_default_context
        ctx = get_default_context()
        return ctx.handle is not None and not ctx.ok()
    except Exception:  # noqa: BLE001
        return False


def _run():
    """Ctrl+C cả lúc đang khởi động (import, init) -> thoát êm, không in traceback."""
    try:
        return main()
    except KeyboardInterrupt:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        print('[tb4_lab04] Đã dừng (Ctrl+C).', flush=True)
        return 1
    except Exception as exc:  # noqa: BLE001
        if type(exc).__name__ != 'ExternalShutdownException' and not _ros_shutting_down():
            raise
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        return 1


if __name__ == '__main__':
    sys.exit(_run())
