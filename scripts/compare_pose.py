#!/usr/bin/env python3
# Bài thực hành 04 – Phần 3: so sánh vị trí robot ƯỚC LƯỢNG trên bản đồ (AMCL)
# với vị trí THẬT trong Gazebo.
#
#   ros2 run tb4_lab04 compare_pose.py                 # in liên tục, Ctrl+C để xem tổng kết
#   ros2 run tb4_lab04 compare_pose.py --duration 60
#
# Nguồn dữ liệu:
#   * Ước lượng : TF map -> base_link (do AMCL publish map->odom, Create 3 publish odom->base_link)
#   * Thực tế   : /sim_ground_truth_pose (nav_msgs/Odometry) – pose của model "turtlebot4"
#                 trong frame world của Gazebo, do create3_sim publish sẵn.
#
# Vì sao so sánh được trực tiếp?
#   Frame "map" của SLAM Toolbox trùng với pose của robot lúc bắt đầu mapping, tức là pose
#   spawn (x, y, yaw) trong world. Script đổi pose thật từ world sang map bằng đúng pose spawn
#   đó (mặc định 0, 0, 0 – trùng với mặc định của launch). Nếu bạn spawn ở chỗ khác thì
#   truyền --spawn-x/--spawn-y/--spawn-yaw giống lúc mapping.

import argparse
import math
import signal
import sys
import time

# ---------------------------------------------------------------------------
# Hàm thuần (có unit test)
# ---------------------------------------------------------------------------


def yaw_from_quaternion(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def wrap_angle(a):
    a = math.fmod(a + math.pi, 2.0 * math.pi)
    if a <= 0.0:
        a += 2.0 * math.pi
    return a - math.pi


def world_to_map(px, py, pyaw, spawn_x, spawn_y, spawn_yaw):
    """Đổi pose từ frame world (Gazebo) sang frame map (gốc = pose spawn)."""
    dx, dy = px - spawn_x, py - spawn_y
    c, s = math.cos(spawn_yaw), math.sin(spawn_yaw)
    return (c * dx + s * dy, -s * dx + c * dy, wrap_angle(pyaw - spawn_yaw))


def pose_error(est, truth):
    """(sai số vị trí m, sai số góc rad – trị tuyệt đối)."""
    return (math.hypot(est[0] - truth[0], est[1] - truth[1]),
            abs(wrap_angle(est[2] - truth[2])))


class ErrorStats:
    def __init__(self):
        self.n = 0
        self.sum_pos = 0.0
        self.max_pos = 0.0
        self.sum_yaw = 0.0
        self.max_yaw = 0.0

    def add(self, pos_err, yaw_err):
        self.n += 1
        self.sum_pos += pos_err
        self.sum_yaw += yaw_err
        self.max_pos = max(self.max_pos, pos_err)
        self.max_yaw = max(self.max_yaw, yaw_err)

    def summary(self):
        if self.n == 0:
            return None
        return {'n': self.n, 'mean_pos': self.sum_pos / self.n, 'max_pos': self.max_pos,
                'mean_yaw': self.sum_yaw / self.n, 'max_yaw': self.max_yaw}


# ---------------------------------------------------------------------------
# Phần ROS
# ---------------------------------------------------------------------------


def main(argv=None):
    parser = argparse.ArgumentParser(description='So sánh pose AMCL với ground truth Gazebo.')
    parser.add_argument('--spawn-x', type=float, default=0.0)
    parser.add_argument('--spawn-y', type=float, default=0.0)
    parser.add_argument('--spawn-yaw', type=float, default=0.0)
    parser.add_argument('--period', type=float, default=1.0, help='chu kỳ in (s, wall)')
    parser.add_argument('--duration', type=float, default=0.0, help='0 = chạy tới khi Ctrl+C')
    parser.add_argument('--tolerance', type=float, default=0.25,
                        help='ngưỡng sai số vị trí trung bình coi là ĐẠT (m)')

    import rclpy
    from rclpy.utilities import remove_ros_args
    args = parser.parse_args(remove_ros_args(argv if argv is not None else sys.argv)[1:])

    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.time import Time
    from nav_msgs.msg import Odometry
    from tf2_ros import TransformException
    from tf2_ros.buffer import Buffer
    from tf2_ros.transform_listener import TransformListener

    rclpy.init(args=argv)
    node = Node('tb4_lab04_compare_pose',
                parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])
    log = node.get_logger()
    tf_buffer = Buffer()
    tf_listener = TransformListener(tf_buffer, node)  # noqa: F841 – phải giữ tham chiếu
    truth = {'msg': None}
    node.create_subscription(Odometry, '/sim_ground_truth_pose',
                             lambda m: truth.__setitem__('msg', m), qos_profile_sensor_data)

    stats = ErrorStats()
    start = time.monotonic()
    next_print = start
    warned = set()

    print('Đang so sánh (Ctrl+C để dừng và xem tổng kết)...')
    print('{:>7} | {:^24} | {:^24} | {:>8} {:>8}'.format(
        't(s)', 'AMCL (x, y, yaw)', 'Gazebo->map (x, y, yaw)', 'e_xy[m]', 'e_yaw[°]'))
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            now = time.monotonic()
            if args.duration > 0 and now - start > args.duration:
                break
            if now < next_print:
                continue
            next_print = now + args.period

            try:
                t = tf_buffer.lookup_transform('map', 'base_link', Time())
                q = t.transform.rotation
                est = (t.transform.translation.x, t.transform.translation.y,
                       yaw_from_quaternion(q.x, q.y, q.z, q.w))
            except TransformException:
                if 'tf' not in warned:
                    log.warn('Chưa có TF map->base_link (AMCL chưa chạy / chưa có initial pose).')
                    warned.add('tf')
                continue
            msg = truth['msg']
            if msg is None:
                if 'gt' not in warned:
                    log.warn('Chưa nhận /sim_ground_truth_pose (chỉ có trong mô phỏng).')
                    warned.add('gt')
                continue
            p, o = msg.pose.pose.position, msg.pose.pose.orientation
            gt = world_to_map(p.x, p.y, yaw_from_quaternion(o.x, o.y, o.z, o.w),
                              args.spawn_x, args.spawn_y, args.spawn_yaw)
            e_pos, e_yaw = pose_error(est, gt)
            stats.add(e_pos, e_yaw)
            print('{:7.1f} | {:7.2f} {:7.2f} {:7.2f}  | {:7.2f} {:7.2f} {:7.2f}  | {:8.3f} {:8.1f}'
                  .format(now - start, est[0], est[1], est[2], gt[0], gt[1], gt[2],
                          e_pos, math.degrees(e_yaw)))
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        s = stats.summary()
        print('\n================ TỔNG KẾT LOCALIZATION ================')
        if s is None:
            print('Không có mẫu nào – kiểm tra AMCL và /sim_ground_truth_pose.')
            code = 1
        else:
            ok = s['mean_pos'] <= args.tolerance
            print('Số mẫu: {n}\nSai số vị trí : TB {mean_pos:.3f} m, lớn nhất {max_pos:.3f} m'
                  .format(**s))
            print('Sai số góc    : TB {:.1f}°, lớn nhất {:.1f}°'.format(
                math.degrees(s['mean_yaw']), math.degrees(s['max_yaw'])))
            print('=> {} (ngưỡng {:.2f} m)'.format('ĐẠT' if ok else 'CHƯA ĐẠT', args.tolerance))
            code = 0 if ok else 1
        print('========================================================')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return code


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
