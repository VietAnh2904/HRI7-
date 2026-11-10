#!/usr/bin/env python3
"""Phần 1 – Kiểm tra định vị: điểm LiDAR (/scan) có TRÙNG tường của bản đồ đã nạp không.

  ros2 run tb4_lab04 localization_check.py               # đo ~10 s rồi kết luận
  ros2 run tb4_lab04 localization_check.py --duration 0  # đo liên tục, Ctrl+C để kết luận

Cách đo: với mỗi scan, chiếu từng tia hợp lệ sang frame map bằng TF (map -> frame của LiDAR, do AMCL
+ odometry cung cấp), tra khoảng cách tới ô TƯỜNG gần nhất của /map. Tia "khớp" nếu điểm cuối cách
tường <= --tolerance (mặc định 0.15 m = 3 ô bản đồ). Tỉ lệ khớp cao = initial pose / AMCL đúng: đó
chính là điều kiện "dữ liệu quét LiDAR trùng khớp với các bức tường trên bản đồ" của đề.
Vật thể mới thêm vào world (người, xe...) không có trên bản đồ -> các tia chạm vào chúng bị tính là
"không khớp", nên ngưỡng ĐẠT là 70 % chứ không phải 100 %.
Bổ sung: in độ lệch chuẩn của AMCL (/amcl_pose). So với vị trí THẬT trong Gazebo: compare_pose.py.
"""

import argparse
import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
import lab_common as lc  # noqa: E402

PASS_RATIO = 0.70


def scan_endpoints(ranges, angle_min, angle_inc, range_min, range_max, step=1):
    """Toạ độ (x, y) điểm cuối các tia hợp lệ trong frame của LiDAR."""
    pts = []
    for k in range(0, len(ranges), step):
        r = ranges[k]
        if not math.isfinite(r) or r < max(range_min, 0.05) or r >= range_max * 0.98:
            continue
        a = angle_min + k * angle_inc
        pts.append((r * math.cos(a), r * math.sin(a)))
    return pts


def match_ratio(dist_field, res, ox, oy, pts_map, tol):
    """(tỉ lệ khớp, khoảng cách trung vị) của các điểm (x, y) trong frame map."""
    import numpy as np
    if not pts_map:
        return None, None
    h, w = dist_field.shape
    ds = []
    for x, y in pts_map:
        i, j = int((x - ox) / res), int((y - oy) / res)
        ds.append(dist_field[j, i] if 0 <= i < w and 0 <= j < h else float('inf'))
    ds = np.array(ds)
    return float(np.mean(ds <= tol)), float(np.median(ds))


def main():
    ap = argparse.ArgumentParser(description='Kiểm tra LiDAR khớp bản đồ (localization)')
    ap.add_argument('--duration', type=float, default=10.0, help='giây đo (0 = tới khi Ctrl+C)')
    ap.add_argument('--tolerance', type=float, default=0.15)
    ap.add_argument('--scan', default='/scan')
    args, _ = ap.parse_known_args()

    import numpy as np
    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
    from rclpy.time import Time
    from rclpy.duration import Duration
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from nav_msgs.msg import OccupancyGrid
    from sensor_msgs.msg import LaserScan
    from tf2_ros import TransformException
    from tf2_ros.buffer import Buffer
    from tf2_ros.transform_listener import TransformListener
    from check_goals import distance_field

    rclpy.init(args=sys.argv)
    node = Node('localization_check',
                parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])
    log = node.get_logger()
    tf_buffer = Buffer()
    tf_listener = TransformListener(tf_buffer, node)  # noqa: F841 (giữ tham chiếu)
    st = {'map': None, 'amcl': None, 'ratios': [], 'medians': [], 'last': 0.0, 'tf_err': None}

    def on_map(msg):
        occ = np.array(msg.data, dtype=np.int16).reshape(msg.info.height, msg.info.width)
        walls = occ >= 65
        st['map'] = (distance_field(walls, msg.info.resolution), msg.info.resolution,
                     msg.info.origin.position.x, msg.info.origin.position.y)
        log.info('Đã nhận /map {}x{} ô, {:.3f} m/ô'.format(msg.info.width, msg.info.height,
                                                          msg.info.resolution))

    def on_amcl(msg):
        st['amcl'] = msg

    def on_scan(msg):
        if st['map'] is None or time.monotonic() - st['last'] < 0.5:
            return
        try:
            t = tf_buffer.lookup_transform('map', msg.header.frame_id, Time.from_msg(msg.header.stamp),
                                           timeout=Duration(seconds=0.2))
        except TransformException:
            # AMCL xử lý /scan_nav (trễ 0.1 s) nên map->odom có thể chưa có cho đúng thời điểm của /scan:
            # dùng TF mới nhất (sai khác không đáng kể khi robot đứng yên / đi chậm).
            try:
                t = tf_buffer.lookup_transform('map', msg.header.frame_id, Time())
            except TransformException as exc:
                st['tf_err'] = str(exc)
                return
        st['last'] = time.monotonic()
        tr, q = t.transform.translation, t.transform.rotation
        yaw = lc.yaw_from_quat(q.x, q.y, q.z, q.w)
        c, s = math.cos(yaw), math.sin(yaw)
        pts = scan_endpoints(msg.ranges, msg.angle_min, msg.angle_increment, msg.range_min, msg.range_max)
        pts_map = [(tr.x + c * x - s * y, tr.y + s * x + c * y) for x, y in pts]
        ratio, med = match_ratio(st['map'][0], st['map'][1], st['map'][2], st['map'][3], pts_map,
                                 args.tolerance)
        if ratio is None:
            return
        st['ratios'].append(ratio)
        st['medians'].append(med)
        log.info('Scan khớp tường: {:5.1f} % ({} tia, lệch trung vị {:.3f} m) | robot (map) ({:.2f}, {:.2f}, '
                 '{:.0f}°)'.format(ratio * 100, len(pts), med, tr.x, tr.y, math.degrees(yaw)))

    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
    node.create_subscription(OccupancyGrid, '/map', on_map, latched)
    node.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', on_amcl, latched)
    node.create_subscription(LaserScan, args.scan, on_scan, qos_profile_sensor_data)

    log.info('Đo trong {} ... (cần: /map từ map_server, TF map->odom từ AMCL, {})'.format(
        '{:.0f} s'.format(args.duration) if args.duration > 0 else 'tới khi Ctrl+C', args.scan))
    start = time.monotonic()
    last_hint = start
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            if args.duration > 0 and time.monotonic() - start > args.duration and st['ratios']:
                break
            if time.monotonic() - last_hint > 10.0 and not st['ratios']:
                last_hint = time.monotonic()
                missing = []
                if st['map'] is None:
                    missing.append('/map')
                if st['tf_err']:
                    missing.append('TF map->LiDAR ({})'.format(st['tf_err'].splitlines()[0][:80]))
                log.warn('Chưa đo được – thiếu: {}'.format(', '.join(missing) or args.scan))
            if args.duration > 0 and time.monotonic() - start > args.duration + 60 and not st['ratios']:
                log.error('Không đo được sau {:.0f} s'.format(time.monotonic() - start))
                return 1
    except KeyboardInterrupt:
        pass
    finally:
        r = st['ratios']
        print('\n============ KIỂM TRA LOCALIZATION ============', flush=True)
        if r:
            mean = sum(r) / len(r)
            print('Số scan đo        : {}'.format(len(r)), flush=True)
            print('Tỉ lệ tia khớp    : TB {:.1f} %, thấp nhất {:.1f} %'.format(mean * 100, min(r) * 100),
                  flush=True)
            print('Lệch trung vị     : {:.3f} m'.format(sum(st['medians']) / len(st['medians'])), flush=True)
            if st['amcl'] is not None:
                cov = st['amcl'].pose.covariance
                print('AMCL độ lệch chuẩn: x {:.2f} m, y {:.2f} m, yaw {:.1f}°'.format(
                    math.sqrt(max(cov[0], 0)), math.sqrt(max(cov[7], 0)),
                    math.degrees(math.sqrt(max(cov[35], 0)))), flush=True)
            ok = mean >= PASS_RATIO
            print('=> {}'.format('ĐẠT: LiDAR trùng tường của bản đồ – initial pose / AMCL đúng' if ok else
                                 'CHƯA ĐẠT: LiDAR lệch bản đồ -> dùng 2D Pose Estimate trên RViz2 đặt lại '
                                 'vị trí robot, kiểm tra cùng world + vị trí spawn như lúc làm bản đồ'),
                  flush=True)
        else:
            print('Không có số liệu.', flush=True)
        print('================================================', flush=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0 if r and sum(r) / len(r) >= PASS_RATIO else 1


if __name__ == '__main__':
    lc.install_ctrl_c_handlers()
    sys.exit(lc.run_main(main, interrupted_code=0))
