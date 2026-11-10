#!/usr/bin/env python3
"""Bài 7 – Gõ câu lệnh thay cho nói (không có micro / kiểm tra phần điều hướng tách khỏi ASR).

  ros2 run tb4_lab04 voice_keyboard.py                      # gõ từng câu, Enter để gửi
  ros2 run tb4_lab04 voice_keyboard.py "Đi đến khu vực A"   # gửi một câu rồi thoát
  ros2 run tb4_lab04 voice_keyboard.py --check "đi tới khu vực Z"   # chỉ phân tích, không gửi (không cần ROS)

Gửi lên /voice/text đúng như speech_recognizer_node.py -> voice_navigator.py xử lý y hệt lệnh nói, và in
phản hồi từ /voice/feedback.
"""

import os
import sys
import threading

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
import lab_common as lc  # noqa: E402


def check(texts):
    import yaml
    from voice_command import ZoneCatalog, parse_command, plan_route
    path = lc.find_config('maps', 'zones.yaml')[0]
    if not path:
        print('Chưa có maps/zones.yaml – chạy zone_segmentation.py')
        return 1
    with open(path, 'r', encoding='utf-8') as f:
        cat = ZoneCatalog(yaml.safe_load(f)['zones'])
    for t in texts:
        c = parse_command(t, cat)
        print('"{}"\n  -> {} | xuất phát: {} | đích: {} | lộ trình nếu robot ở nơi khác: {}\n  -> {}'.format(
            t, c.intent, c.start, c.targets, plan_route(c, None), c.message))
    return 0


def main():
    args = [a for a in sys.argv[1:] if not a.startswith('--ros-args')]
    if args and args[0] == '--check':
        return check(args[1:])
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    rclpy.init(args=sys.argv)
    node = Node('tb4_voice_keyboard')
    pub = node.create_publisher(String, 'voice/text', 10)
    node.create_subscription(String, 'voice/feedback', lambda m: print('  << ' + m.data, flush=True), 10)
    try:
        texts = [a for a in args if not a.startswith('-')]
        if texts:
            import time
            end = time.monotonic() + 3.0
            while pub.get_subscription_count() == 0 and time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=0.1)
            for t in texts:
                pub.publish(String(data=t))
                print('>> ' + t, flush=True)
            end = time.monotonic() + 2.0
            while time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=0.1)
            return 0
        spin = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
        spin.start()
        print('Gõ câu lệnh (Enter để gửi, Ctrl+D để thoát). Ví dụ: Đi đến khu vực A', flush=True)
        for line in sys.stdin:
            line = line.strip()
            if line:
                pub.publish(String(data=line))
        return 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    lc.install_ctrl_c_handlers()
    sys.exit(lc.run_main(main, interrupted_code=0))
