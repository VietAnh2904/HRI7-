#!/usr/bin/env python3
"""Phát một chuỗi cử chỉ giả lên /detected_gesture – thử robot_behavior_node KHÔNG cần webcam.

  ros2 run tb4_lab04 gesture_sequence.py --ros-args -p sequence:="Pointing_Up:1.5,None:1,Thumb_Up:1.5,None:25"
  ros2 run tb4_lab04 gesture_sequence.py --ros-args -p preset:=all      # demo đủ 6 cử chỉ (~2.5 phút)

Mỗi phần tử "Tên:giây" được gửi liên tục 20 Hz (giống gesture_recognition_node). Hết chuỗi thì NGỪNG gửi
-> robot_behavior_node mất tín hiệu -> robot dừng (đúng thiết kế). Chỉ dùng để kiểm thử – video demo của
bài phải dùng webcam thật.
"""

import signal
import sys
import time

PRESETS = {
    # chọn -> xác nhận -> chờ xong, lần lượt từng hành vi; có xoè tay (dừng) và nắm tay (huỷ)
    'all': ('None:2,Wave:1.5,None:1,Thumb_Up:1.2,None:9,'
            'Pointing_Up:1.2,None:1,Thumb_Up:1.2,None:26,'
            'Victory:1.2,None:1,Thumb_Up:1.2,None:8,Open_Palm:1,None:2,Thumb_Up:1.2,None:26,'
            'Pointing_Up:1.2,None:1,Thumb_Up:1.2,None:3,Closed_Fist:1,None:3'),
    'stop': 'None:2,Victory:1.2,None:1,Thumb_Up:1.2,None:5,Open_Palm:1.5,None:3',
    'lost': 'None:2,Pointing_Up:1.2,None:1,Thumb_Up:1.2,None:4',
}


def parse_sequence(text):
    out = []
    for item in str(text).split(','):
        item = item.strip()
        if not item:
            continue
        name, _, sec = item.partition(':')
        out.append((name.strip(), float(sec) if sec else 1.0))
    return out


def main(argv=None):
    import rclpy
    from rclpy.node import Node
    from rclpy.signals import SignalHandlerOptions
    from std_msgs.msg import String

    rclpy.init(args=argv, signal_handler_options=SignalHandlerOptions.NO)
    node = Node('gesture_sequence')
    preset = node.declare_parameter('preset', '').value
    seq_text = node.declare_parameter('sequence', '').value or PRESETS.get(preset, '')
    rate = float(node.declare_parameter('rate', 20.0).value)
    topic = node.declare_parameter('topic', '/detected_gesture').value
    if not seq_text:
        node.get_logger().error('Cần -p sequence:="Ten:giay,..." hoặc -p preset:={}'.format('|'.join(PRESETS)))
        return 2
    seq = parse_sequence(seq_text)
    pub = node.create_publisher(String, topic, 10)
    stop = {'flag': False}
    signal.signal(signal.SIGINT, lambda *a: stop.__setitem__('flag', True))
    time.sleep(1.0)                       # cho DDS kịp nối
    try:
        for name, sec in seq:
            node.get_logger().info('-> {} ({:.1f} s)'.format(name, sec))
            end = time.monotonic() + sec
            while time.monotonic() < end and not stop['flag']:
                pub.publish(String(data=name))
                rclpy.spin_once(node, timeout_sec=0.0)
                time.sleep(1.0 / rate)
            if stop['flag']:
                break
        node.get_logger().info('Hết chuỗi – ngừng gửi (robot_behavior_node sẽ báo mất tín hiệu)')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
