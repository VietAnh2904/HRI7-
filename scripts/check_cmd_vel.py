#!/usr/bin/env python3
"""Bài 6 – Phần 1: kiểm tra TurtleBot 4 (Gazebo) nhận lệnh vận tốc qua /cmd_vel.

  ros2 run tb4_lab04 check_cmd_vel.py
  ros2 run tb4_lab04 check_cmd_vel.py --ros-args -p undock:=false

Các bước: chờ /odom -> (robot đang ở dock thì gọi /undock) -> gửi /cmd_vel quay trái 0.5 rad/s trong
3 s, quay phải 3 s, tiến 0.1 m/s 2 s, lùi 2 s -> so sánh /odom trước/sau -> ĐẠT nếu robot quay > 40° và
tiến > 0.1 m. Robot kết thúc gần đúng chỗ cũ.
"""

import math
import signal
import sys
import time


def main(argv=None):
    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.signals import SignalHandlerOptions

    rclpy.init(args=argv, signal_handler_options=SignalHandlerOptions.NO)
    node = Node('check_cmd_vel')
    log = node.get_logger()
    undock = node.declare_parameter('undock', True).value
    stop = {'flag': False}
    signal.signal(signal.SIGINT, lambda *a: stop.__setitem__('flag', True))
    st = {'pose': None, 'docked': None, 'clock': None, 'clock_wall': 0.0}

    # Thời lượng các bước tính theo GIỜ MÔ PHỎNG (/clock) nếu có: máy yếu (Gazebo chậm hơn thời gian thực)
    # vẫn quay / đi đúng quãng dự kiến. Không có /clock (robot thật) -> giờ thực.
    from rosgraph_msgs.msg import Clock

    def on_clock(msg):
        st['clock'] = msg.clock.sec + msg.clock.nanosec * 1e-9
        st['clock_wall'] = time.monotonic()

    node.create_subscription(Clock, '/clock', on_clock, qos_profile_sensor_data)

    def now():
        if st['clock'] is not None and time.monotonic() - st['clock_wall'] < 2.0:
            return st['clock']
        return time.monotonic()

    def on_odom(msg):
        p = msg.pose.pose
        q = p.orientation
        st['pose'] = (p.position.x, p.position.y,
                      math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))

    node.create_subscription(Odometry, '/odom', on_odom, qos_profile_sensor_data)
    pub = node.create_publisher(Twist, '/cmd_vel', 10)
    try:
        from irobot_create_msgs.msg import DockStatus
        node.create_subscription(DockStatus, '/dock_status',
                                 lambda m: st.__setitem__('docked', bool(m.is_docked)), qos_profile_sensor_data)
    except ImportError:
        DockStatus = None

    def spin_wall(sec):
        end = time.monotonic() + sec
        while time.monotonic() < end and not stop['flag']:
            rclpy.spin_once(node, timeout_sec=0.05)

    def spin_for(sec, max_wall=600.0):
        t0, w0 = now(), time.monotonic()
        while now() - t0 < sec and time.monotonic() - w0 < max_wall and not stop['flag']:
            rclpy.spin_once(node, timeout_sec=0.05)

    def drive(lin, ang, sec):
        tw = Twist()
        tw.linear.x, tw.angular.z = float(lin), float(ang)
        t0, w0 = now(), time.monotonic()
        while now() - t0 < sec and time.monotonic() - w0 < 600.0 and not stop['flag']:
            pub.publish(tw)
            rclpy.spin_once(node, timeout_sec=0.05)
        pub.publish(Twist())

    code = 1
    try:
        log.info('Chờ /odom ...')
        t0 = time.monotonic()
        while st['pose'] is None and time.monotonic() - t0 < 60 and not stop['flag']:
            spin_wall(0.2)
        if st['pose'] is None:
            log.error('Không có /odom sau 60 s – mô phỏng chưa chạy? (ros2 launch tb4_lab04 gesture_sim.launch.py)')
            return 1
        spin_for(1.5)
        if undock and st['docked']:
            from rclpy.action import ActionClient
            from irobot_create_msgs.action import Undock
            cli = ActionClient(node, Undock, '/undock')
            log.info('Robot đang ở dock -> /undock')
            if not cli.wait_for_server(timeout_sec=20.0):
                log.error('Không thấy action /undock')
                return 1
            fut = cli.send_goal_async(Undock.Goal())
            while not fut.done() and not stop['flag']:
                spin_wall(0.1)
            res = fut.result().get_result_async()
            t1 = now()
            while not res.done() and now() - t1 < 90 and not stop['flag']:
                spin_wall(0.2)
            if not res.done():
                log.error('Undock quá 90 s (giờ mô phỏng)')
                return 1
            log.info('Undock xong')
        p0 = st['pose']
        log.info('Pose trước: x={:.2f} y={:.2f} yaw={:.0f}°'.format(p0[0], p0[1], math.degrees(p0[2])))
        log.info('/cmd_vel: quay trái 0.5 rad/s, 3 s')
        drive(0.0, 0.5, 3.0)
        spin_for(0.5)
        p1 = st['pose']
        rot = abs(math.degrees(math.atan2(math.sin(p1[2] - p0[2]), math.cos(p1[2] - p0[2]))))
        log.info('  -> đã quay {:.0f}°'.format(rot))
        log.info('/cmd_vel: quay phải 0.5 rad/s, 3 s')
        drive(0.0, -0.5, 3.0)
        spin_for(0.5)
        p2 = st['pose']
        log.info('/cmd_vel: tiến 0.1 m/s, 2 s')
        drive(0.1, 0.0, 2.0)
        spin_for(0.5)
        p3 = st['pose']
        fwd = math.hypot(p3[0] - p2[0], p3[1] - p2[1])
        log.info('  -> đã đi {:.2f} m'.format(fwd))
        log.info('/cmd_vel: lùi 0.1 m/s, 2 s')
        drive(-0.1, 0.0, 2.0)
        spin_for(0.5)
        p4 = st['pose']
        log.info('Pose sau: x={:.2f} y={:.2f} yaw={:.0f}° (lệch {:.2f} m so với trước)'.format(
            p4[0], p4[1], math.degrees(p4[2]), math.hypot(p4[0] - p0[0], p4[1] - p0[1])))
        ok = rot > 40.0 and fwd > 0.1
        log.info('KẾT QUẢ: {} – robot {} lệnh /cmd_vel'.format('ĐẠT' if ok else 'KHÔNG ĐẠT',
                                                            'nhận' if ok else 'KHÔNG nhận đúng'))
        code = 0 if ok else 1
    finally:
        for _ in range(3):
            pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return code


if __name__ == '__main__':
    sys.exit(main())
