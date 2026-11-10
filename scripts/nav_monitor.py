#!/usr/bin/env python3
# Bài thực hành 04 – Phần 4: theo dõi các Nav2 Goal đặt từ RViz2 (công cụ "Nav2 Goal").
#
#   ros2 run tb4_lab04 nav_monitor.py
#
# Script KHÔNG điều khiển robot. Nó chỉ lắng nghe và ghi lại bằng chứng cho báo cáo/video:
#   * mỗi goal: lúc nhận, kết quả (SUCCEEDED / ABORTED / CANCELED), thời gian, số lần recovery,
#     quãng đường còn lại lúc kết thúc
#   * va chạm: sự kiện BUMP từ /hazard_detection (bumper của Create 3) trong lúc đang có goal
#   * điều khiển tay: cảnh báo nếu có node teleop đang publish /cmd_vel trong lúc có goal
# Nhấn Ctrl+C để in bảng tổng kết.

import signal
import sys
import time

STATUS_NAMES = {0: 'UNKNOWN', 1: 'ACCEPTED', 2: 'EXECUTING', 3: 'CANCELING',
                4: 'SUCCEEDED', 5: 'CANCELED', 6: 'ABORTED'}
TERMINAL = {4, 5, 6}


def goal_id_str(uuid_bytes):
    """UUID goal (mảng 16 byte) -> chuỗi hex ngắn."""
    return ''.join('{:02x}'.format(int(b)) for b in uuid_bytes)[:8]


class GoalBook:
    """Theo dõi vòng đời goal từ GoalStatusArray (thuần Python, có unit test)."""

    def __init__(self):
        self.goals = {}   # id -> dict
        self.order = []

    def update(self, gid, status, now):
        """Trả về danh sách sự kiện (gid, trạng_thái_cũ, trạng_thái_mới)."""
        events = []
        g = self.goals.get(gid)
        if g is None:
            g = {'status': None, 't_start': now, 't_end': None, 'bumps': 0,
                 'recoveries': 0, 'remaining': None, 'manual_warned': False}
            self.goals[gid] = g
            self.order.append(gid)
        if g['status'] != status:
            events.append((gid, g['status'], status))
            g['status'] = status
            if status in TERMINAL and g['t_end'] is None:
                g['t_end'] = now
        return events

    def active(self):
        return [gid for gid in self.order if self.goals[gid]['status'] in (1, 2, 3)]

    def summary(self):
        counts = {'SUCCEEDED': 0, 'ABORTED': 0, 'CANCELED': 0, 'OTHER': 0}
        for gid in self.order:
            name = STATUS_NAMES.get(self.goals[gid]['status'], 'OTHER')
            counts[name if name in counts else 'OTHER'] += 1
        return counts


def main(argv=None):
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy,
                           qos_profile_sensor_data)
    from action_msgs.msg import GoalStatusArray
    from geometry_msgs.msg import PolygonStamped
    from irobot_create_msgs.msg import HazardDetection, HazardDetectionVector
    from nav2_msgs.action import NavigateToPose
    from rosgraph_msgs.msg import Clock

    rclpy.init(args=argv)
    node = Node('tb4_lab04_nav_monitor')
    log = node.get_logger()
    book = GoalBook()
    t0 = time.monotonic()

    def stamp():
        return time.monotonic() - t0

    # QoS giống status publisher của action server (reliable + transient local)
    status_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10,
                            reliability=ReliabilityPolicy.RELIABLE,
                            durability=DurabilityPolicy.TRANSIENT_LOCAL)

    def on_status(msg):
        now = stamp()
        for st in msg.status_list:
            gid = goal_id_str(st.goal_info.goal_id.uuid)
            for _, old, new in book.update(gid, st.status, now):
                g = book.goals[gid]
                if old is None and new in TERMINAL:
                    # goal cũ đã kết thúc trước khi script chạy – chỉ ghi nhận
                    log.info('Goal {} (cũ): {}'.format(gid, STATUS_NAMES.get(new, new)))
                elif new in (1, 2) and old is None:
                    log.info('>>> Goal {} được nhận – robot bắt đầu tự lập kế hoạch'.format(gid))
                elif new in TERMINAL:
                    dur = (g['t_end'] or now) - g['t_start']
                    rem = '' if g['remaining'] is None else ', còn {:.2f} m'.format(g['remaining'])
                    text = '<<< Goal {}: {} sau {:.1f} s (recovery: {}, bump: {}{})'.format(
                        gid, STATUS_NAMES.get(new, new), dur, g['recoveries'], g['bumps'], rem)
                    # Không dùng `fn = log.info if ... else log.warn`: rclpy cấm đổi mức log
                    # tại cùng một dòng lệnh giữa các lần gọi.
                    if new == 4:
                        log.info(text)
                    else:
                        log.warn(text)

    def on_feedback(msg):
        gid = goal_id_str(msg.goal_id.uuid)
        g = book.goals.get(gid)
        if g is not None:
            g['recoveries'] = msg.feedback.number_of_recoveries
            g['remaining'] = msg.feedback.distance_remaining

    hz = {'bumping': False}

    def on_hazard(msg):
        # /hazard_detection lặp lại liên tục khi bumper còn bị ép -> chỉ đếm SƯỜN LÊN
        # (mỗi lần va chạm mới tính 1 lần).
        bumping = any(d.type == HazardDetection.BUMP for d in msg.detections)
        rising = bumping and not hz['bumping']
        hz['bumping'] = bumping
        active = book.active()
        if rising and active:
            for gid in active:
                book.goals[gid]['bumps'] += 1
            log.error('VA CHẠM (bumper) trong lúc thực hiện goal {}!'.format(', '.join(active)))

    # Phát hiện Nav2 "đứng hình": local costmap ngừng cập nhật vị trí robot (published_footprint)
    # trong khi giờ mô phỏng vẫn chạy. Thường do lỗi deadlock đã biết của tf2_ros MessageFilter
    # trên ROS 2 Humble khi máy quá tải -> log lặp "Transform data too old ... map to odom".
    fp = {'stamp': None, 'clock': None, 'warned': False}

    def on_footprint(msg):
        fp['stamp'] = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if fp['clock'] is not None and fp['clock'] - fp['stamp'] < 5.0:
            fp['warned'] = False   # đã hồi phục -> cho phép cảnh báo lại nếu treo lần nữa

    def on_clock(msg):
        fp['clock'] = msg.clock.sec + msg.clock.nanosec * 1e-9

    def check_nav2_alive():
        if not book.active() or fp['stamp'] is None or fp['clock'] is None or fp['warned']:
            return
        lag = fp['clock'] - fp['stamp']
        if lag > 15.0:
            fp['warned'] = True
            log.error('Nav2 có vẻ ĐỨNG HÌNH: local costmap không cập nhật vị trí robot đã {:.0f} s '
                      '(giờ mô phỏng). Đây là lỗi treo TF đã biết của ROS 2 Humble khi máy quá '
                      'tải. Cách xử lý: Ctrl+C terminal navigation, chạy '
                      '`ros2 run tb4_lab04 kill_sim.sh`, launch lại (đóng bớt ứng dụng nặng).'
                      .format(lag))

    def check_manual_control():
        active = book.active()
        if not active:
            return
        names = [i.node_name for i in node.get_publishers_info_by_topic('/cmd_vel')]
        manual = [n for n in names if 'teleop' in n or 'joy' in n]
        for gid in active:
            g = book.goals[gid]
            if manual and not g['manual_warned']:
                g['manual_warned'] = True
                log.warn('Đang có node điều khiển tay publish /cmd_vel ({}) trong lúc chạy goal '
                         '{} – yêu cầu đề bài là KHÔNG điều khiển tay. Hãy tắt teleop.'.format(
                             ', '.join(manual), gid))

    node.create_subscription(GoalStatusArray, '/navigate_to_pose/_action/status', on_status,
                             status_qos)
    node.create_subscription(NavigateToPose.Impl.FeedbackMessage,
                             '/navigate_to_pose/_action/feedback', on_feedback, 10)
    node.create_subscription(HazardDetectionVector, '/hazard_detection', on_hazard,
                             qos_profile_sensor_data)
    node.create_subscription(PolygonStamped, '/local_costmap/published_footprint', on_footprint,
                             10)
    node.create_subscription(Clock, '/clock', on_clock, 10)
    node.create_timer(1.0, check_manual_control)
    node.create_timer(2.0, check_nav2_alive)

    log.info('Đang theo dõi /navigate_to_pose. Đặt goal bằng nút "Nav2 Goal" trên RViz2. '
             'Ctrl+C để xem tổng kết.')
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        c = book.summary()
        print('\n================ TỔNG KẾT NAVIGATION ================')
        for gid in book.order:
            g = book.goals[gid]
            dur = '' if g['t_end'] is None else '{:.1f} s'.format(g['t_end'] - g['t_start'])
            print('  Goal {}  {:<10} {:>8}  recovery={} bump={}'.format(
                gid, STATUS_NAMES.get(g['status'], g['status']), dur, g['recoveries'],
                g['bumps']))
        total_bumps = sum(g['bumps'] for g in book.goals.values())
        print('Thành công: {SUCCEEDED} | Thất bại: {ABORTED} | Huỷ: {CANCELED}'.format(**c))
        print('Va chạm (bumper) trong lúc chạy goal: {}'.format(total_bumps))
        print('=> {}'.format('ĐẠT (>= 3 goal thành công, không va chạm)'
                             if c['SUCCEEDED'] >= 3 and total_bumps == 0 else
                             'CHƯA ĐẠT yêu cầu ">= 3 goal thành công, không va chạm"'))
        print('======================================================')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


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
