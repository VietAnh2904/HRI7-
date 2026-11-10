#!/usr/bin/env python3
"""Đánh dấu dock sạc là VẬT CẢN cho costmap của Nav2.

Vì sao cần? Dock của Create 3 cao ~0.1 m, còn LiDAR của TurtleBot 4 quét ở độ cao cao hơn đỉnh dock
-> LiDAR (và SLAM, costmap) KHÔNG nhìn thấy dock. Nav2 có thể lập đường đi xuyên qua dock (ví dụ khi
quay về điểm xuất phát), robot đâm vào dock, bumper kích hoạt phản xạ và goal thất bại.

Node này publish một đám điểm (PointCloud2, topic /dock_obstacle, frame MAP) bao phần thân đứng
của dock; nav2.yaml có một lớp costmap riêng `dock_layer` chỉ đọc topic này (marking, không clearing)
nên LiDAR không xoá được nó, và inflation_layer tạo vùng đệm quanh dock.

Vì sao frame MAP, không phải odom? Bản đầu neo dock theo odom: odom TRÔI dần theo quãng đường (đo
được: sau 1 vòng ~7 m, dock ảo lệch khỏi dock thật đủ để Nav2 lái robot vào góc dock, kẹt, bánh trượt
làm odom lệch thêm 0.75 m). Frame map được AMCL/SLAM hiệu chỉnh liên tục và TRÙNG điểm spawn (cả lúc
mapping lẫn navigation – điều kiện bắt buộc của bài, xem README mục 0), nên dock luôn ở
(0.157, 0) trong map.

Vị trí dock so với điểm spawn: TF TĨNH odom->std_dock_link do mô phỏng Create 3 publish (odom trùng
spawn ở thời điểm đầu) – đọc thẳng từ /tf_static (không
dùng TransformListener: nó phải xử lý cả luồng /tf tần số cao, tốn CPU vô ích – đo được ~8 % CPU). Nếu
không có TF đó, dùng giá trị mặc định của mô phỏng: dock đặt trước điểm spawn 0.157 m, quay mặt về robot.
"""

import math
import signal
import sys


DEFAULT_DOCK = (0.157, 0.0, math.pi)   # create3_spawn / turtlebot4_spawn: x+0.157, yaw+pi
# Hộp va chạm của dock (standard_dock.urdf.xacro): tâm (-0.045, 0, 0.05), kích thước 0.04 x 0.13 x 0.1
# trong frame std_dock_link (+x hướng về phía robot), tức x -0.065..-0.025, |y| <= 0.065.
# Đánh dấu RỘNG HƠN có chủ đích: lần thử đầu đánh dấu |y| <= 0.08 thì vùng cấm của Nav2 chỉ tới
# 0.08 + 0.175 (bán kính robot) = 0.255 m; robot đi sát mép đó và CHẠM góc dock (bumper báo va chạm).
# |y| <= 0.16 -> vùng cấm tới ~0.335 m, mép robot cách dock >= ~7 cm kể cả khi rời rạc hoá ô 5 cm.
BOX_X = (-0.11, -0.02)
BOX_Y = (-0.16, 0.16)
STEP = 0.02


def dock_points(dx, dy, dyaw, z=0.05):
    """Các điểm (x, y, z) phủ thân dock đặt tại (dx, dy, dyaw), cùng frame với (dx, dy)."""
    c, s = math.cos(dyaw), math.sin(dyaw)
    pts = []
    # Chia đều, LUÔN gồm cả hai mép (bước <= STEP) – tránh hụt mép khi kích thước không chia hết.
    nx = int(math.ceil((BOX_X[1] - BOX_X[0]) / STEP - 1e-9)) + 1
    ny = int(math.ceil((BOX_Y[1] - BOX_Y[0]) / STEP - 1e-9)) + 1
    for i in range(nx):
        for j in range(ny):
            lx = BOX_X[0] + (BOX_X[1] - BOX_X[0]) * i / (nx - 1)
            ly = BOX_Y[0] + (BOX_Y[1] - BOX_Y[0]) * j / (ny - 1)
            pts.append((dx + c * lx - s * ly, dy + s * lx + c * ly, z))
    return pts


def main():
    # Import ROS trong main(): Ctrl+C ngay lúc node đang khởi động cũng được _run() bắt êm.
    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import PointCloud2
    from sensor_msgs_py import point_cloud2
    from std_msgs.msg import Header
    from tf2_msgs.msg import TFMessage

    rclpy.init(args=sys.argv)
    node = Node('tb4_lab04_dock_obstacle',
                parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])
    node.declare_parameter('rate', 2.0)
    node.declare_parameter('frame', 'map')     # frame publish (map: không trôi theo odom)
    frame = node.get_parameter('frame').value
    pub = node.create_publisher(PointCloud2, '/dock_obstacle', 10)
    state = {'pts': None, 'tries': 0, 'tf': None}

    def on_static(msg):
        for t in msg.transforms:
            # TF tĩnh odom->std_dock_link = vị trí dock so với điểm spawn (= gốc map).
            if t.header.frame_id.lstrip('/') == 'odom' and \
                    t.child_frame_id.lstrip('/').endswith('std_dock_link'):
                state['tf'] = t

    static_qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,
                            durability=DurabilityPolicy.TRANSIENT_LOCAL)
    sub = node.create_subscription(TFMessage, '/tf_static', on_static, static_qos)

    def tick():
        if state['pts'] is None:
            state['tries'] += 1
            t = state['tf']
            if t is not None:
                q = t.transform.rotation
                yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
                pose = (t.transform.translation.x, t.transform.translation.y, yaw)
                node.get_logger().info('Dock (TF tĩnh odom->std_dock_link) tại ({:.3f}, {:.3f}) – '
                                       'publish trong frame {}'.format(pose[0], pose[1], frame))
            elif state['tries'] < 20:        # chờ ~10 s cho TF tĩnh
                return
            else:
                pose = DEFAULT_DOCK
                node.get_logger().warn('Không có TF odom->std_dock_link -> dùng vị trí dock mặc định '
                                       '({:.3f}, {:.3f}) trong frame {}'.format(pose[0], pose[1], frame))
            state['pts'] = dock_points(*pose)
            node.destroy_subscription(sub)   # không cần nghe /tf_static nữa
        header = Header(frame_id=frame, stamp=node.get_clock().now().to_msg())
        pub.publish(point_cloud2.create_cloud_xyz32(header, state['pts']))

    node.create_timer(1.0 / max(0.2, node.get_parameter('rate').value), tick)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        # Ctrl+C trong terminal + launch gửi SIGINT 2 lần: bỏ qua lần 2 để thoát sạch (mã 0)
        # thay vì log đỏ "process has died ... exit code -2".
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


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
    """Ctrl+C ở bất kỳ lúc nào (kể cả khi đang khởi động) -> thoát mã 0, không traceback."""
    try:
        main()
    except KeyboardInterrupt:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
    except Exception as exc:  # noqa: BLE001
        if type(exc).__name__ != 'ExternalShutdownException' and not _ros_shutting_down():
            raise
        signal.signal(signal.SIGINT, signal.SIG_IGN)
    return 0


if __name__ == '__main__':
    sys.exit(_run())
