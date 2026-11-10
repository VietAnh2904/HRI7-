#!/usr/bin/env python3
# Bài thực hành 04 – chờ mô phỏng THỰC SỰ chạy rồi mới bật SLAM / localization / Nav2.
#
# Được mapping.launch.py và navigation.launch.py gọi tự động (không cần chạy tay).
# Điều kiện "sẵn sàng": /clock đang tăng và /odom đã có (tức diffdrive_controller đã chạy).
#
# Vì sao không chỉ chờ N giây? Trên máy chậm Gazebo có thể mất vài phút mới khởi động xong.
# Bật SLAM/AMCL/Nav2 trong lúc đó làm CPU quá tải: lifecycle manager báo
# "Server amcl was unable to be reached after 4.00s by bond", và plugin gz_ros2_control
# (chính thức) có thể treo khi hỏi robot_description -> Gazebo đứng, không có /clock.

import signal
import sys
import time

TIMEOUT_DEFAULT = 300.0


def main(argv=None):
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rosgraph_msgs.msg import Clock
    from nav_msgs.msg import Odometry

    rclpy.init(args=argv)
    node = Node('tb4_lab04_wait_for_sim')
    log = node.get_logger()
    timeout = node.declare_parameter('timeout', TIMEOUT_DEFAULT).value
    st = {'clock': None, 'clock_first': None, 'odom': False}

    def on_clock(msg):
        t = msg.clock.sec + msg.clock.nanosec * 1e-9
        if st['clock_first'] is None:
            st['clock_first'] = t
        st['clock'] = t

    def on_odom(_msg):
        st['odom'] = True

    node.create_subscription(Clock, '/clock', on_clock, qos_profile_sensor_data)
    node.create_subscription(Odometry, '/odom', on_odom, qos_profile_sensor_data)

    start = time.monotonic()
    next_report = start + 15.0
    code = 1
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.2)
            ready = (st['odom'] and st['clock'] is not None
                     and st['clock'] - st['clock_first'] >= 2.0)
            if ready:
                log.info('Mô phỏng đã chạy (sim time = {:.1f} s, có /odom) -> bật bước tiếp '
                         'theo.'.format(st['clock']))
                code = 0
                break
            now = time.monotonic()
            if now - start > timeout:
                log.error(
                    'Sau {:.0f} s mô phỏng vẫn chưa sẵn sàng ({}). Nếu log Gazebo dừng ở '
                    '"asking for robot_description": đó là lỗi treo đã biết của gz_ros2_control '
                    'khi máy quá tải -> Ctrl+C, chạy `ros2 run tb4_lab04 kill_sim.sh` rồi '
                    'launch lại. Vẫn tiếp tục khởi chạy bước sau.'.format(
                        timeout, 'không có /clock' if st['clock'] is None else 'chưa có /odom'))
                break
            if now >= next_report:
                next_report = now + 15.0
                missing = []
                if st['clock'] is None:
                    missing.append('/clock')
                if not st['odom']:
                    missing.append('/odom')
                log.info('Đang chờ mô phỏng khởi động... (chưa có: {}) {:.0f} s'.format(
                    ', '.join(missing) or 'đủ 2 s sim', now - start))
    except (KeyboardInterrupt, ExternalShutdownException):
        code = 0   # người dùng Ctrl+C: tắt êm, không để launch báo "[ERROR] process has died"
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)   # SIGINT lần 2 (launch) -> bỏ qua
        if code != 0 and _ros_shutting_down():   # rclpy tắt context trước khi kịp ném KeyboardInterrupt
            code = 0
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
    """Ctrl+C ở bất kỳ lúc nào (kể cả khi đang import/khởi động) -> thoát êm, không traceback."""
    try:
        return main()
    except KeyboardInterrupt:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        return 1
    except Exception as exc:  # noqa: BLE001
        if type(exc).__name__ != 'ExternalShutdownException' and not _ros_shutting_down():
            raise
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        return 1


if __name__ == '__main__':
    sys.exit(_run())
