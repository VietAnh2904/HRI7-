#!/usr/bin/env python3
"""Kiểm tra dữ liệu cảm biến của TurtleBot 4 mô phỏng có HỢP LÝ không (không chỉ "có topic").

    ros2 run tb4_lab04 check_sensors.py            # chạy khi sim.launch.py / mapping / explore đang chạy
    ros2 run tb4_lab04 check_sensors.py --time 10  # đo lâu hơn (giây mô phỏng)

Vì sao cần? Trong Gazebo Fortress, LiDAR (rplidar) VÀ 4 cảm biến cliff (chống rơi) của Create 3
đều là cảm biến kiểu `gpu_lidar`: khoảng cách được tính bằng RENDER đồ hoạ. Nếu driver đồ hoạ/GPU
render sai, topic vẫn chạy đều nhưng số đo vô nghĩa (LƯU Ý: robot hiện màu đen trong Gazebo KHÔNG phải
dấu hiệu chắc chắn – khi render bằng CPU robot cũng đen mà cảm biến vẫn đúng; hãy tin script này), dẫn tới:
  * LiDAR trả toàn giá trị ~range_min (0.16 m) -> SLAM chỉ vẽ được một ô vuông ~0.35 m, check_sim
    báo "phía trước không thoáng", Nav2/khám phá đứng im;
  * cliff báo "hụt chân" liên tục -> Create 3 chạy phản xạ (reflex) ngay khi robot vừa nhúc nhích,
    huỷ action undock (kết quả CANCELED), robot vẫn đứng sát dock.
Script đọc /scan, 4 topic cliff nội bộ, /hazard_detection, /dock_status và kết luận từng mục.
Mã thoát 0 = mọi thứ hợp lý, 1 = có vấn đề.
"""

import argparse
import math
import signal
import sys
import time

CLIFF_SENSORS = ('cliff_front_left', 'cliff_front_right', 'cliff_side_left', 'cliff_side_right')
CLIFF_THRESHOLD = 0.03   # irobot_create_ignition_toolbox/cliff.cpp: min(range) > 0.03 m -> CLIFF
HAZARD_NAMES = {0: 'BACKUP_LIMIT', 1: 'BUMP', 2: 'CLIFF', 3: 'STALL', 4: 'WHEEL_DROP',
                5: 'OBJECT_PROXIMITY'}
# Hazard làm Create 3 chạy phản xạ (reflex_behavior.cpp). BACKUP_LIMIT/OBJECT_PROXIMITY thì không.
REFLEX_HAZARDS = ('BUMP', 'CLIFF', 'STALL', 'WHEEL_DROP')


def scan_health(ranges, range_min, range_max):
    """Đánh giá một LaserScan. Trả về dict có 'verdict' trong:
    ok | empty | no_returns (gần như toàn inf/NaN) | near_min (>= 50 % tia ~range_min, tức "mù")
    | all_close (không tia nào xa quá 0.6 m)."""
    n = len(ranges)
    hits = [r for r in ranges if math.isfinite(r) and range_min <= r <= range_max]
    near = [r for r in hits if r <= range_min + 0.05]
    s = sorted(hits)
    out = {
        'n': n,
        'valid': len(hits),
        'frac_valid': (len(hits) / float(n)) if n else 0.0,
        'frac_near_min': (len(near) / float(len(hits))) if hits else 0.0,
        'min': s[0] if s else float('nan'),
        'median': s[len(s) // 2] if s else float('nan'),
        'max': s[-1] if s else float('nan'),
    }
    if n == 0:
        v = 'empty'
    elif out['frac_valid'] < 0.05:
        v = 'no_returns'
    elif out['frac_near_min'] >= 0.5:
        v = 'near_min'
    elif out['max'] < 0.6:
        v = 'all_close'
    else:
        v = 'ok'
    out['verdict'] = v
    return out


def cliff_min(ranges):
    """Giống irobot_create_toolbox::FindMinimumRange (std::min_element, NaN bị bỏ qua cho an toàn)."""
    vals = [r for r in ranges if not math.isnan(r)]
    return min(vals) if vals else float('inf')


def cliff_triggered(ranges):
    return cliff_min(ranges) > CLIFF_THRESHOLD


SCAN_ADVICE = {
    'empty': 'LaserScan rỗng.',
    'no_returns': 'gần như mọi tia đều inf/NaN – trong mê cung điều này không thể xảy ra.',
    'near_min': 'phần lớn tia bằng ~range_min – LiDAR "mù" (render độ sâu hỏng).',
    'all_close': 'không tia nào xa quá 0.6 m – LiDAR gần như chắc chắn đo sai.',
}

RENDER_FIX = """Nếu CHỈ cảm biến cliff sai: các launch của tb4_lab04 đã mặc định tắt phản xạ cliff
(cliff_reflex:=false) nên undock/lái vẫn chạy được – đừng thêm cliff_reflex:=true.
Nếu LiDAR sai (hoặc vẫn lỗi): driver đồ hoạ/GPU render sai. Mỗi lần thử: Ctrl+C, kill_sim.sh, rồi:
  1) Ép render bằng CPU (chậm hơn nhưng đúng – hay cần trên WSL2): thêm software_render:=true vào lệnh
     launch, ví dụ:  ros2 launch tb4_lab04 sim.launch.py software_render:=true
     (tương đương export LIBGL_ALWAYS_SOFTWARE=1 trước khi launch)
  2) Laptop 2 card (Intel + NVIDIA, Ubuntu cài thật): ép dùng card NVIDIA
       export __NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia
  rồi chạy lại launch TRONG CÙNG terminal đó và chạy lại check_sensors.py.
  Kiểm tra card đang dùng: glxinfo -B | grep -E "OpenGL renderer|OpenGL version"."""


def main(argv=None):
    ap = argparse.ArgumentParser(description='Kiểm tra dữ liệu LiDAR/cliff/hazard có hợp lý không')
    ap.add_argument('--time', type=float, default=5.0, help='thời gian đo (giây mô phỏng), mặc định 5')
    ap.add_argument('--timeout', type=float, default=120.0, help='giới hạn giờ thực (s)')
    a = ap.parse_args([x for x in (argv if argv is not None else sys.argv[1:])
                       if not x.startswith('__') and x != '--ros-args'])

    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan
    try:
        from irobot_create_msgs.msg import DockStatus, HazardDetectionVector
    except ImportError:
        DockStatus = HazardDetectionVector = None

    rclpy.init()
    node = Node('tb4_lab04_check_sensors',
                parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])
    data = {'scan': None, 'scan_n': 0, 'dock': None, 'hz': {}, 'hz_msgs': 0}
    cliff = {s: {'n': 0, 'trig': 0, 'last': None} for s in CLIFF_SENSORS}

    def on_scan(m):
        data['scan'] = m
        data['scan_n'] += 1

    node.create_subscription(LaserScan, '/scan', on_scan, qos_profile_sensor_data)
    for s in CLIFF_SENSORS:
        def cb(m, s=s):
            c = cliff[s]
            c['n'] += 1
            c['last'] = cliff_min(m.ranges)
            if cliff_triggered(m.ranges):
                c['trig'] += 1
        node.create_subscription(LaserScan, '/_internal/{}/scan'.format(s), cb,
                                 qos_profile_sensor_data)
    if HazardDetectionVector is not None:
        def on_hz(m):
            data['hz_msgs'] += 1
            for d in m.detections:
                k = HAZARD_NAMES.get(d.type, str(d.type))
                data['hz'][k] = data['hz'].get(k, 0) + 1
        node.create_subscription(HazardDetectionVector, '/hazard_detection', on_hz,
                                 qos_profile_sensor_data)
        node.create_subscription(DockStatus, '/dock_status',
                                 lambda m: data.__setitem__('dock', m), qos_profile_sensor_data)

    wall_end = time.monotonic() + a.timeout
    # chờ đồng hồ mô phỏng chạy
    while rclpy.ok() and node.get_clock().now().nanoseconds == 0 and time.monotonic() < wall_end:
        rclpy.spin_once(node, timeout_sec=0.2)
    t0 = node.get_clock().now().nanoseconds
    while rclpy.ok() and time.monotonic() < wall_end and \
            (node.get_clock().now().nanoseconds - t0) * 1e-9 < a.time:
        rclpy.spin_once(node, timeout_sec=0.1)

    bad = False
    print('================ KIỂM TRA CẢM BIẾN ================')
    if t0 == 0:
        print('[FAIL] Không có /clock – mô phỏng chưa chạy (hoặc terminal này không thấy các node khác).')
        print('       Kiểm tra cùng ROS_LOCALHOST_ONLY/ROS_DOMAIN_ID giữa các terminal.')
        node.destroy_node()
        rclpy.shutdown()
        return 1

    m = data['scan']
    if m is None:
        print('[FAIL] /scan: không nhận được tin nào.')
        bad = True
    else:
        h = scan_health(m.ranges, m.range_min, m.range_max)
        tag = 'OK  ' if h['verdict'] == 'ok' else 'FAIL'
        print('[{}] /scan: {} tin; {} tia, hợp lệ {:.0f} %, gần range_min {:.0f} %, '
              'min/trung vị/max = {:.2f}/{:.2f}/{:.2f} m (range_min {:.3f})'.format(
                  tag, data['scan_n'], h['n'], 100 * h['frac_valid'], 100 * h['frac_near_min'],
                  h['min'], h['median'], h['max'], m.range_min))
        if h['verdict'] != 'ok':
            print('       -> ' + SCAN_ADVICE[h['verdict']])
            bad = True

    cliff_bad = []
    for s in CLIFF_SENSORS:
        c = cliff[s]
        if c['n'] == 0:
            print('[WARN] {}: không nhận được tin (topic /_internal/{}/scan).'.format(s, s))
            continue
        frac = c['trig'] / float(c['n'])
        ok = frac < 0.5
        print('[{}] {}: {} tin, báo "hụt chân" {:.0f} % (min range gần nhất {:.4f} m, ngưỡng {} m)'
              .format('OK  ' if ok else 'FAIL', s, c['n'], 100 * frac, c['last'], CLIFF_THRESHOLD))
        if not ok:
            cliff_bad.append(s)
    if cliff_bad:
        bad = True
        print('       -> robot đang đứng trên sàn mà cliff vẫn báo hụt chân: Create 3 sẽ chạy phản xạ '
              'ngay khi robot nhúc nhích, huỷ undock (CANCELED) và bỏ qua /cmd_vel.')

    if HazardDetectionVector is not None:
        if data['hz_msgs'] == 0:
            print('[WARN] /hazard_detection: không nhận được tin.')
        else:
            reflex = {k: v for k, v in data['hz'].items() if k in REFLEX_HAZARDS}
            other = {k: v for k, v in data['hz'].items() if k not in REFLEX_HAZARDS}
            if reflex:
                print('[WARN] /hazard_detection: có hazard gây phản xạ {} (trên {} tin).'.format(
                    reflex, data['hz_msgs']))
                print('       Đứng yên mà vẫn có BUMP/CLIFF liên tục là bất thường; nếu robot vừa va '
                      'tường thì bình thường.')
            else:
                print('[OK  ] /hazard_detection: không có hazard gây phản xạ{}.'.format(
                    ' (chỉ có {} – bình thường, không gây phản xạ)'.format(other) if other else ''))
        d = data['dock']
        if d is not None:
            print('[INFO] /dock_status: is_docked={} dock_visible={}'.format(d.is_docked,
                                                                           d.dock_visible))

    print('====================================================')
    if bad:
        print('KẾT LUẬN: cảm biến cho số đo KHÔNG hợp lý – SLAM/Nav2/undock sẽ không chạy đúng.')
        print(RENDER_FIX)
    else:
        print('KẾT LUẬN: số đo LiDAR và cliff hợp lý.')
    node.destroy_node()
    rclpy.shutdown()
    return 1 if bad else 0


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
    """Ctrl+C ở BẤT KỲ giai đoạn nào (kể cả lúc còn đang chờ hệ thống khởi động) -> thoát êm, không
    in traceback; bỏ qua SIGINT lần 2 (terminal + launch cùng gửi)."""
    try:
        return main()
    except KeyboardInterrupt:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        print('[tb4_lab04] Đã dừng (Ctrl+C).', flush=True)
        return 1
    except Exception as exc:  # noqa: BLE001
        if type(exc).__name__ == 'ExternalShutdownException' or _ros_shutting_down():   # đang tắt (Ctrl+C)
            signal.signal(signal.SIGINT, signal.SIG_IGN)
            return 1
        raise


if __name__ == '__main__':
    sys.exit(_run())
