#!/usr/bin/env python3
"""Vẽ quỹ đạo robot của một lần chạy robot_behavior_node (ảnh cho báo cáo Bài 6).

  ros2 run tb4_lab04 plot_gesture_run.py                       # lần chạy mới nhất trong ~/tb4_results
  ros2 run tb4_lab04 plot_gesture_run.py ~/tb4_results/gesture_20261025_101500 -o traj.png

Đọc odom.csv + events.csv -> hình: quỹ đạo (x, y) tô màu theo hành vi đang chạy, mốc cử chỉ; và bảng
tóm tắt sự kiện in ra terminal.
"""

import argparse
import csv
import glob
import math
import os
import sys

COLORS = {'greet': '#e6a700', 'forward_back': '#1f77b4', 'circle': '#d62728', 'setup': '#7f7f7f'}
LABELS = {'greet': 'Chào (xoay trái-phải)', 'forward_back': 'Tiến-lùi 4 lần', 'circle': 'Quỹ đạo tròn',
          'setup': 'Undock + ra giữa sân'}


def latest_run(root):
    runs = sorted(glob.glob(os.path.join(os.path.expanduser(root), 'gesture_*')))
    runs = [r for r in runs if os.path.isfile(os.path.join(r, 'odom.csv'))]
    return runs[-1] if runs else None


def read_csv(path):
    if not os.path.isfile(path):
        return []
    with open(path, newline='', encoding='utf-8') as f:
        return list(csv.DictReader(f))


def segments(rows):
    """Tách odom thành các đoạn liên tiếp cùng 'active' (hành vi đang chạy)."""
    out = []
    cur, pts = None, []
    for r in rows:
        a = r.get('active') or ''
        p = (float(r['x']), float(r['y']))
        if a != cur:
            if pts:
                out.append((cur, pts))
            cur, pts = a, ([pts[-1]] if pts else [])
        pts.append(p)
    if pts:
        out.append((cur, pts))
    return out


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument('run', nargs='?', default=None, help='thư mục gesture_<thời_gian>')
    ap.add_argument('-o', '--output', default=None, help='file ảnh (mặc định <run>/trajectory.png)')
    args, _ = ap.parse_known_args(argv)
    run = args.run or latest_run(os.environ.get('TB4_RESULTS_DIR', '~/tb4_results'))
    if not run or not os.path.isdir(os.path.expanduser(run)):
        print('Không tìm thấy thư mục kết quả (chạy robot_behavior_node trước).')
        return 1
    run = os.path.expanduser(run)
    odom = read_csv(os.path.join(run, 'odom.csv'))
    events = read_csv(os.path.join(run, 'events.csv'))
    if not odom:
        print('odom.csv trống: {}'.format(run))
        return 1
    print('Lần chạy: {} – {} điểm odom, {} sự kiện'.format(run, len(odom), len(events)))
    for e in events:
        if e['type'] in ('start', 'stop', 'setup') or (e['type'] == 'gesture' and e['gesture'] != 'None'):
            print('  t={:>7}s  {:<8} {:<12} {:<11} {}'.format(e['t'], e['type'], e['gesture'], e['state'],
                                                            e['detail']))
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except ImportError:
        print('Thiếu matplotlib: sudo apt install python3-matplotlib')
        return 1
    fig, ax = plt.subplots(figsize=(7.5, 6.5))
    shown = set()
    for active, pts in segments(odom):
        xs, ys = [p[0] for p in pts], [p[1] for p in pts]
        if active in COLORS:
            lab = LABELS[active] if active not in shown else None
            shown.add(active)
            if max(xs) - min(xs) < 0.05 and max(ys) - min(ys) < 0.05:
                # quay tại chỗ (chào): không có đường đi -> vẽ một điểm đánh dấu
                ax.plot([sum(xs) / len(xs)], [sum(ys) / len(ys)], 'o', color=COLORS[active], ms=13,
                        mfc='none', mew=2.5, label=lab)
            else:
                ax.plot(xs, ys, '-', color=COLORS[active], lw=2.2, label=lab)
        else:
            ax.plot(xs, ys, '-', color='#bbbbbb', lw=1.0)
    x0, y0 = float(odom[0]['x']), float(odom[0]['y'])
    x1, y1, th1 = float(odom[-1]['x']), float(odom[-1]['y']), float(odom[-1]['yaw'])
    ax.plot([x0], [y0], 'ks', ms=8, label='Bắt đầu (dock)')
    ax.annotate('', xy=(x1 + 0.25 * math.cos(th1), y1 + 0.25 * math.sin(th1)), xytext=(x1, y1),
                arrowprops=dict(arrowstyle='->', color='k', lw=2))
    ax.plot([x1], [y1], 'ko', ms=6, label='Kết thúc')
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)
    ax.set_xlabel('x (m) – frame odom')
    ax.set_ylabel('y (m)')
    ax.set_title('Quỹ đạo TurtleBot 4 điều khiển bằng cử chỉ tay')
    ax.legend(loc='best', fontsize=9)
    out = args.output or os.path.join(run, 'trajectory.png')
    fig.tight_layout()
    fig.savefig(out, dpi=150)
    print('Đã vẽ: {}'.format(out))
    return 0


if __name__ == '__main__':
    sys.exit(main())
