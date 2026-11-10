#!/usr/bin/env python3
"""Phần 2 – Khảo sát và ghi lại toạ độ mục tiêu (x, y, yaw) trong frame map.

  ros2 run tb4_lab04 goal_recorder.py
  ros2 run tb4_lab04 goal_recorder.py --output ~/my_goals.yaml

Hai cách lấy điểm (đúng 2 gợi ý của đề):
  * Nút "Publish Point" trên RViz2 rồi click lên bản đồ  -> topic /clicked_point (geometry_msgs/PointStamped).
    Điểm click không có hướng: yaw được đặt = hướng từ điểm trước tới điểm này (hướng robot sẽ đi tới),
    điểm đầu tiên yaw = 0 – sửa lại trong file nếu muốn robot quay mặt về phía vật thể.
  * Nhấn Enter trong terminal này -> lấy pose HIỆN TẠI của robot từ /amcl_pose (có cả yaw).
Lệnh trong terminal: Enter = ghi pose robot | u = xoá điểm cuối | w = ghi file | q = ghi file và thoát.
Ctrl+C cũng in đoạn YAML để dán vào config/goals.yaml. Kiểm tra: ros2 run tb4_lab04 check_goals.py
"""

import argparse
import math
import os
import sys
import threading

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
import lab_common as lc  # noqa: E402


def heading(prev, cur):
    if prev is None:
        return 0.0
    return math.atan2(cur[1] - prev[1], cur[0] - prev[0])


def to_yaml(points, initial=(0.0, 0.0, 0.0)):
    lines = ['# Ghi bằng goal_recorder.py – frame map, yaw (rad)',
             'initial_pose: {{x: {:.2f}, y: {:.2f}, yaw: {:.2f}}}'.format(*initial), '', 'goals:']
    for k, (x, y, yaw, src) in enumerate(points, 1):
        lines += ['  - name: "Goal {} ({})"'.format(k, src),
                  '    x: {:.2f}'.format(x),
                  '    y: {:.2f}'.format(y),
                  '    yaw: {:.3f}    # {:.0f} độ'.format(yaw, math.degrees(yaw))]
    return '\n'.join(lines) + '\n'


def main():
    ap = argparse.ArgumentParser(description='Ghi toạ độ goal từ RViz2 (/clicked_point) hoặc /amcl_pose')
    ap.add_argument('--output', default=None,
                    help='file YAML (mặc định ~/tb4_results/goals_recorded.yaml)')
    args, _ = ap.parse_known_args()
    out = os.path.expanduser(args.output) if args.output else os.path.join(lc.results_dir(),
                                                                             'goals_recorded.yaml')

    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from geometry_msgs.msg import PointStamped, PoseWithCovarianceStamped

    rclpy.init(args=sys.argv)
    node = Node('goal_recorder', parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])
    log = node.get_logger()
    points = []
    state = {'amcl': None, 'quit': False}
    lock = threading.Lock()

    def add(x, y, yaw, src):
        with lock:
            points.append((x, y, yaw, src))
            n = len(points)
        log.info('Goal {}: x={:.2f}  y={:.2f}  yaw={:.3f} rad ({:.0f}°)  [{}]'.format(
            n, x, y, yaw, math.degrees(yaw), src))

    def on_click(msg):
        if msg.header.frame_id and msg.header.frame_id.lstrip('/') != 'map':
            log.warn('Điểm click ở frame "{}" (không phải map) – đặt Fixed Frame của RViz2 = map'.format(
                msg.header.frame_id))
        with lock:
            prev = points[-1] if points else None
        p = (msg.point.x, msg.point.y)
        add(p[0], p[1], heading(prev, p), 'Publish Point')

    def on_amcl(msg):
        q = msg.pose.pose.orientation
        state['amcl'] = (msg.pose.pose.position.x, msg.pose.pose.position.y,
                         lc.yaw_from_quat(q.x, q.y, q.z, q.w))

    node.create_subscription(PointStamped, 'clicked_point', on_click, 10)
    node.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', on_amcl,
                             QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                        reliability=ReliabilityPolicy.RELIABLE))

    def write():
        with lock:
            text = to_yaml(points)
        os.makedirs(os.path.dirname(out) or '.', exist_ok=True)
        with open(out, 'w', encoding='utf-8') as f:
            f.write(text)
        log.info('Đã ghi {} điểm vào {}'.format(len(points), out))

    def stdin_loop():
        while not state['quit']:
            try:
                line = sys.stdin.readline()
            except (OSError, ValueError):
                return
            if line == '':
                return                     # stdin đóng (chạy từ launch) -> chỉ dùng Publish Point
            cmd = line.strip().lower()
            if cmd == '':
                if state['amcl'] is None:
                    log.warn('Chưa có /amcl_pose (AMCL chưa chạy / chưa có initial pose)')
                else:
                    add(*state['amcl'], 'amcl_pose')
            elif cmd == 'u':
                with lock:
                    removed = points.pop() if points else None
                log.info('Đã xoá điểm cuối' if removed else 'Không còn điểm nào')
            elif cmd == 'w':
                write()
            elif cmd == 'q':
                write()
                state['quit'] = True
            else:
                log.info('Lệnh: Enter = ghi pose robot | u = xoá điểm cuối | w = ghi file | q = thoát')

    threading.Thread(target=stdin_loop, daemon=True).start()
    log.info('Bấm "Publish Point" trên RViz2 rồi click lên bản đồ, hoặc nhấn Enter để lấy pose robot '
             '(/amcl_pose). u = xoá, w = ghi file, q = ghi + thoát. File: {}'.format(out))
    try:
        while rclpy.ok() and not state['quit']:
            rclpy.spin_once(node, timeout_sec=0.2)
    finally:
        if points:
            print('\n# ---- Dán vào config/goals.yaml ----\n' + to_yaml(points), flush=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    lc.install_ctrl_c_handlers()
    sys.exit(lc.run_main(main, interrupted_code=0))
