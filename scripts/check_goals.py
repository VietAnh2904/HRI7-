#!/usr/bin/env python3
"""Kiểm tra goal (config/goals.yaml) và vật thể (config/objects.yaml) trên bản đồ – KHÔNG cần ROS chạy.

  ros2 run tb4_lab04 check_goals.py                 # kiểm tra, in bảng kết quả
  ros2 run tb4_lab04 check_goals.py --plot          # + vẽ ảnh bản đồ kèm goal/vật thể/đường đi
  ros2 run tb4_lab04 check_goals.py --plot --run ~/tb4_results/nav_<thời_gian>/run.json
                                                    # + quỹ đạo thật và đường Nav2 đã lập (ảnh báo cáo)

Mỗi goal được kiểm tra:
  1. nằm trong bản đồ, trên ô TRỐNG đã biết (không phải vùng xám / tường);
  2. cách vật cản (tường + vật thể) >= --min-clearance (mặc định 0.45 m = bán kính robot 0.175 m +
     vùng đệm) – goal sát tường là nguyên nhân số 1 của "Failed to make progress";
  3. đi tới được từ goal trước (goal 1: từ chỗ robot sau khi undock) qua hành lang đủ rộng cho robot
     – tìm đường A* trên bản đồ đã nới bán kính robot (ước lượng, không phải planner của Nav2);
  4. camera (OAK-D, FOV ngang 60°) đặt tại goal nhìn thấy vật thể nào (trong FOV, không bị tường che).
Mã thoát: 0 = không có lỗi, 1 = có lỗi.

Chỉ dùng numpy + PyYAML (có sẵn với ROS 2). Vẽ ảnh cần matplotlib (sudo apt install python3-matplotlib).
"""

import argparse
import csv
import heapq
import json
import math
import os
import sys

import numpy as np
import yaml

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
import lab_common as lc  # noqa: E402

ROBOT_RADIUS = 0.175          # TurtleBot 4 (nav2.yaml: robot_radius)
CAMERA_HFOV = 1.047           # oakd.urdf.xacro: horizontal_fov
CAMERA_RANGE = (0.5, 8.0)     # khoảng cách YOLO còn nhận được vật cỡ người với ảnh 320x240
UNDOCK_BACKUP = 0.40          # undock: robot lùi ~0.4 m rồi quay 180°
# Dock (thấp hơn LiDAR, không có trên bản đồ) – cùng hộp với scripts/dock_obstacle.py, frame dock
DOCK_OFFSET = 0.157
DOCK_BOX = (-0.11, -0.02, -0.16, 0.16)


# ------------------------------------------------------------------------------------------ bản đồ
def read_pgm(path):
    """Đọc ảnh PGM (P5 nhị phân hoặc P2 văn bản) -> mảng uint8 (hàng 0 = mép TRÊN ảnh)."""
    with open(path, 'rb') as f:
        data = f.read()
    tokens, pos = [], 0
    while len(tokens) < 4:
        while pos < len(data) and data[pos:pos + 1].isspace():
            pos += 1
        if data[pos:pos + 1] == b'#':
            while pos < len(data) and data[pos:pos + 1] not in (b'\n', b'\r'):
                pos += 1
            continue
        start = pos
        while pos < len(data) and not data[pos:pos + 1].isspace():
            pos += 1
        tokens.append(data[start:pos])
    magic, w, h, maxval = tokens[0], int(tokens[1]), int(tokens[2]), int(tokens[3])
    if magic == b'P5':
        if maxval > 255:
            raise ValueError('{}: PGM 16 bit không được hỗ trợ'.format(path))
        pix = np.frombuffer(data[pos + 1:pos + 1 + w * h], dtype=np.uint8)
        if pix.size != w * h:
            raise ValueError('{}: thiếu dữ liệu ảnh ({} / {} byte)'.format(path, pix.size, w * h))
        img = pix.reshape(h, w)
    elif magic == b'P2':
        vals = np.array(data[pos:].split()[:w * h], dtype=np.int32)
        img = (vals.reshape(h, w) * (255.0 / maxval)).astype(np.uint8)
    else:
        raise ValueError('{}: không phải ảnh PGM (magic {!r})'.format(path, magic))
    return img


class GridMap:
    """Occupancy grid như nav2_map_server đọc (mode trinary): 100 = vật cản, 0 = trống, -1 = chưa biết.
    Hàng 0 của `occ` ở y NHỎ NHẤT (giống nav_msgs/OccupancyGrid)."""

    def __init__(self, yaml_path):
        with open(yaml_path, 'r', encoding='utf-8') as f:
            meta = yaml.safe_load(f)
        image = str(meta['image'])
        if not os.path.isabs(image):
            image = os.path.join(os.path.dirname(os.path.abspath(yaml_path)), image)
        img = read_pgm(image)[::-1].astype(np.float64)
        self.res = float(meta['resolution'])
        self.ox, self.oy = float(meta['origin'][0]), float(meta['origin'][1])
        self.origin_yaw = float(meta['origin'][2]) if len(meta['origin']) > 2 else 0.0
        negate = int(meta.get('negate', 0))
        occ_t = float(meta.get('occupied_thresh', 0.65))
        free_t = float(meta.get('free_thresh', 0.25))
        p = img / 255.0 if negate else (255.0 - img) / 255.0
        self.occ = np.full(img.shape, -1, dtype=np.int8)
        self.occ[p > occ_t] = 100
        self.occ[p < free_t] = 0
        self.h, self.w = self.occ.shape
        self.path = yaml_path

    def to_cell(self, x, y):
        return int(math.floor((x - self.ox) / self.res)), int(math.floor((y - self.oy) / self.res))

    def to_world(self, i, j):
        return self.ox + (i + 0.5) * self.res, self.oy + (j + 0.5) * self.res

    def inside(self, i, j):
        return 0 <= i < self.w and 0 <= j < self.h

    def extent(self):
        return [self.ox, self.ox + self.w * self.res, self.oy, self.oy + self.h * self.res]

    def cell_centers(self):
        xs = self.ox + (np.arange(self.w) + 0.5) * self.res
        ys = self.oy + (np.arange(self.h) + 0.5) * self.res
        return np.meshgrid(xs, ys)


def footprint_mask(gm, obj, margin=0.0):
    """Mảng bool: các ô bị vật thể chiếm (footprint trong config/objects.yaml)."""
    X, Y = gm.cell_centers()
    dx, dy = X - float(obj['x']), Y - float(obj['y'])
    fp = obj.get('footprint') or {'radius': 0.3}
    if 'box' in fp:
        x0, x1, y0, y1 = [float(v) for v in fp['box']]
        c, s = math.cos(float(obj.get('yaw', 0.0))), math.sin(float(obj.get('yaw', 0.0)))
        lx, ly = c * dx + s * dy, -s * dx + c * dy           # về frame của model
        return (lx >= x0 - margin) & (lx <= x1 + margin) & (ly >= y0 - margin) & (ly <= y1 + margin)
    r = float(fp.get('radius', 0.3)) + margin
    return dx * dx + dy * dy <= r * r


def footprint_points(obj, n=12):
    """Vài điểm trên footprint (tâm + mép) để xét camera thấy MỘT PHẦN vật thể lớn."""
    x, y, yaw = float(obj['x']), float(obj['y']), float(obj.get('yaw', 0.0))
    fp = obj.get('footprint') or {'radius': 0.3}
    pts = []
    if 'box' in fp:
        x0, x1, y0, y1 = [float(v) for v in fp['box']]
        c, s = math.cos(yaw), math.sin(yaw)
        local = [((x0 + x1) / 2, (y0 + y1) / 2)]
        for t in np.linspace(0.0, 1.0, 4):
            local += [(x0 + (x1 - x0) * t, y0), (x0 + (x1 - x0) * t, y1),
                      (x0, y0 + (y1 - y0) * t), (x1, y0 + (y1 - y0) * t)]
        pts = [(x + c * lx - s * ly, y + s * lx + c * ly) for lx, ly in local]
    else:
        r = float(fp.get('radius', 0.3))
        pts = [(x, y)] + [(x + r * math.cos(a), y + r * math.sin(a))
                          for a in np.linspace(0, 2 * math.pi, n, endpoint=False)]
    return pts


def dock_mask(gm, pose):
    X, Y = gm.cell_centers()
    x0, y0, yaw = pose
    dx_, dy_ = x0 + DOCK_OFFSET * math.cos(yaw), y0 + DOCK_OFFSET * math.sin(yaw)
    dyaw = yaw + math.pi                                      # dock quay mặt về robot
    c, s = math.cos(dyaw), math.sin(dyaw)
    lx, ly = c * (X - dx_) + s * (Y - dy_), -s * (X - dx_) + c * (Y - dy_)
    bx0, bx1, by0, by1 = DOCK_BOX
    return (lx >= bx0) & (lx <= bx1) & (ly >= by0) & (ly <= by1)


def distance_field(blocked, res):
    """Khoảng cách (m) từ mỗi ô tới ô `blocked` gần nhất. Dùng scipy nếu có, nếu không: chamfer 3-4."""
    try:
        from scipy import ndimage
        return ndimage.distance_transform_edt(~blocked) * res
    except ImportError:
        pass
    INF = 10 ** 9
    h, w = blocked.shape
    d = np.where(blocked, 0, INF).astype(np.int64)
    for j in range(h):                                    # lượt xuôi
        for i in range(w):
            v = d[j, i]
            if v == 0:
                continue
            if i > 0:
                v = min(v, d[j, i - 1] + 3)
            if j > 0:
                v = min(v, d[j - 1, i] + 3)
                if i > 0:
                    v = min(v, d[j - 1, i - 1] + 4)
                if i < w - 1:
                    v = min(v, d[j - 1, i + 1] + 4)
            d[j, i] = v
    for j in range(h - 1, -1, -1):                        # lượt ngược
        for i in range(w - 1, -1, -1):
            v = d[j, i]
            if v == 0:
                continue
            if i < w - 1:
                v = min(v, d[j, i + 1] + 3)
            if j < h - 1:
                v = min(v, d[j + 1, i] + 3)
                if i < w - 1:
                    v = min(v, d[j + 1, i + 1] + 4)
                if i > 0:
                    v = min(v, d[j + 1, i - 1] + 4)
            d[j, i] = v
    return d / 3.0 * res


def astar(passable, start, goal):
    """A* 8 hướng trên lưới. Trả về danh sách ô (i, j) hoặc None."""
    h, w = passable.shape
    (si, sj), (gi, gj) = start, goal
    if not (passable[sj, si] and passable[gj, gi]):
        return None
    sq2 = math.sqrt(2.0)
    moves = [(1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
             (1, 1, sq2), (1, -1, sq2), (-1, 1, sq2), (-1, -1, sq2)]
    g = {start: 0.0}
    parent = {start: None}
    heap = [(math.hypot(gi - si, gj - sj), 0.0, start)]
    while heap:
        _, cost, cur = heapq.heappop(heap)
        if cur == goal:
            path = []
            while cur is not None:
                path.append(cur)
                cur = parent[cur]
            return path[::-1]
        if cost > g.get(cur, float('inf')):
            continue
        ci, cj = cur
        for di, dj, step in moves:
            ni, nj = ci + di, cj + dj
            if not (0 <= ni < w and 0 <= nj < h) or not passable[nj, ni]:
                continue
            if di and dj and not (passable[cj, ni] and passable[nj, ci]):
                continue                                   # không cắt góc
            nc = cost + step
            if nc < g.get((ni, nj), float('inf')):
                g[(ni, nj)] = nc
                parent[(ni, nj)] = cur
                heapq.heappush(heap, (nc + math.hypot(gi - ni, gj - nj), nc, (ni, nj)))
    return None


def nearest_passable(passable, cell, max_r=10):
    i0, j0 = cell
    h, w = passable.shape
    best = None
    for r in range(max_r + 1):
        for j in range(j0 - r, j0 + r + 1):
            for i in range(i0 - r, i0 + r + 1):
                if 0 <= i < w and 0 <= j < h and passable[j, i]:
                    d = (i - i0) ** 2 + (j - j0) ** 2
                    if best is None or d < best[0]:
                        best = (d, (i, j))
        if best is not None:
            return best[1]
    return None


def line_of_sight(gm, walls, p0, p1, ignore=None):
    """True nếu đoạn p0->p1 không cắt ô tường (bỏ qua ô trong `ignore`)."""
    n = int(max(2, math.hypot(p1[0] - p0[0], p1[1] - p0[1]) / (gm.res * 0.5)))
    for t in np.linspace(0.0, 1.0, n):
        i, j = gm.to_cell(p0[0] + (p1[0] - p0[0]) * t, p0[1] + (p1[1] - p0[1]) * t)
        if not gm.inside(i, j):
            return False
        if walls[j, i] and not (ignore is not None and ignore[j, i]):
            return False
    return True


def camera_view(gm, walls, pose, obj, obj_mask, hfov=CAMERA_HFOV, rng=CAMERA_RANGE):
    """(thấy?, khoảng cách tới tâm, góc lệch so với hướng camera (rad), ghi chú)."""
    gx, gy, gyaw = pose
    ox, oy = float(obj['x']), float(obj['y'])
    dist = math.hypot(ox - gx, oy - gy)
    bearing = lc.normalize_angle(math.atan2(oy - gy, ox - gx) - gyaw)
    seen = []
    for px, py in footprint_points(obj):
        b = lc.normalize_angle(math.atan2(py - gy, px - gx) - gyaw)
        d = math.hypot(px - gx, py - gy)
        if abs(b) <= hfov / 2 and rng[0] <= d <= rng[1] and \
                line_of_sight(gm, walls, (gx, gy), (px, py), ignore=obj_mask):
            seen.append((px, py))
    if not seen:
        if dist > rng[1]:
            note = 'quá xa ({:.1f} m)'.format(dist)
        elif abs(bearing) > hfov / 2:
            note = 'ngoài FOV'
        else:
            note = 'bị che'
        return False, dist, bearing, note
    center_in = abs(bearing) <= hfov / 2
    return True, dist, bearing, ('giữa khung' if center_in else 'một phần')


# --------------------------------------------------------------------------------------- cấu hình
def load_goals(path):
    """Đọc goals.yaml -> (initial_pose (x, y, yaw), [dict(name, x, y, yaw)]). Lỗi -> ValueError."""
    with open(path, 'r', encoding='utf-8') as f:
        data = yaml.safe_load(f) or {}
    if not isinstance(data, dict):
        raise ValueError('{}: phải là YAML dạng key: value'.format(path))

    def pose_of(d, where):
        if not isinstance(d, dict):
            raise ValueError('{}: {} phải có dạng {{x: .., y: .., yaw: ..}}'.format(path, where))
        try:
            x, y = float(d['x']), float(d['y'])
        except (KeyError, TypeError, ValueError):
            raise ValueError('{}: {} thiếu x / y dạng số'.format(path, where))
        if 'yaw_deg' in d:
            yaw = math.radians(float(d['yaw_deg']))
        else:
            yaw = float(d.get('yaw', 0.0))
        if not all(math.isfinite(v) for v in (x, y, yaw)):
            raise ValueError('{}: {} có giá trị không hợp lệ'.format(path, where))
        return x, y, lc.normalize_angle(yaw)

    init = pose_of(data.get('initial_pose', {'x': 0.0, 'y': 0.0, 'yaw': 0.0}), 'initial_pose')
    raw = data.get('goals')
    if not isinstance(raw, list) or not raw:
        raise ValueError('{}: thiếu danh sách "goals:"'.format(path))
    goals = []
    for k, g in enumerate(raw, 1):
        x, y, yaw = pose_of(g, 'goal #{}'.format(k))
        name = str(g.get('name') or 'Goal {}'.format(k))
        goals.append({'name': name, 'x': x, 'y': y, 'yaw': yaw})
    return init, goals


def load_objects(path):
    with open(path, 'r', encoding='utf-8') as f:
        data = yaml.safe_load(f) or {}
    objs = data.get('objects') or []
    for o in objs:
        for key in ('name', 'x', 'y'):
            if key not in o:
                raise ValueError('{}: vật thể thiếu "{}": {}'.format(path, key, o))
    return objs


# ------------------------------------------------------------------------------------------ kiểm tra
def analyse(gm, init, goals, objects, min_clearance=0.45, robot_radius=ROBOT_RADIUS):
    """Trả về dict: issues [(mức, thông điệp)], legs [...], views {goal_idx: [...]}, ..."""
    issues = []
    walls = gm.occ == 100
    obj_masks = {o['name']: footprint_mask(gm, o) for o in objects}
    blocked = (gm.occ != 0).copy()
    for m in obj_masks.values():
        blocked |= m
    blocked |= dock_mask(gm, init)
    clear = distance_field(blocked, gm.res)
    passable = clear >= robot_radius + 0.05            # robot + 5 cm (ô 5 cm rời rạc hoá)

    for o in objects:
        i, j = gm.to_cell(float(o['x']), float(o['y']))
        if not gm.inside(i, j):
            issues.append(('LỖI', 'Vật thể {} nằm ngoài bản đồ'.format(o['name'])))
        elif (obj_masks[o['name']] & walls).any():
            issues.append(('CẢNH BÁO', 'Vật thể {} chồng lên tường trên bản đồ'.format(o['name'])))

    ux = init[0] - UNDOCK_BACKUP * math.cos(init[2])
    uy = init[1] - UNDOCK_BACKUP * math.sin(init[2])
    start = (ux, uy, lc.normalize_angle(init[2] + math.pi))
    legs, views = [], {}
    prev = start
    for k, g in enumerate(goals):
        label = 'Goal {} ({})'.format(k + 1, g['name'])
        i, j = gm.to_cell(g['x'], g['y'])
        cl = None
        if not gm.inside(i, j):
            issues.append(('LỖI', '{}: ({:.2f}, {:.2f}) nằm ngoài bản đồ'.format(label, g['x'], g['y'])))
        elif gm.occ[j, i] != 0:
            kind = 'tường/vật cản' if gm.occ[j, i] == 100 else 'vùng CHƯA BIẾT (xám)'
            issues.append(('LỖI', '{}: nằm trên {}'.format(label, kind)))
        else:
            cl = float(clear[j, i])
            if cl < robot_radius:
                issues.append(('LỖI', '{}: chỉ cách vật cản {:.2f} m (< bán kính robot {:.3f} m)'.format(
                    label, cl, robot_radius)))
            elif cl < min_clearance:
                issues.append(('CẢNH BÁO', '{}: cách vật cản {:.2f} m (< {:.2f} m) – Nav2 dễ báo '
                               '"Failed to make progress"'.format(label, cl, min_clearance)))
        # đường đi ước lượng
        a = nearest_passable(passable, gm.to_cell(prev[0], prev[1]))
        b = nearest_passable(passable, (i, j)) if gm.inside(i, j) else None
        path = astar(passable, a, b) if a is not None and b is not None else None
        if path is None:
            issues.append(('LỖI', '{}: KHÔNG đi tới được từ điểm trước (hành lang hẹp hơn robot hoặc '
                           'bị vật thể chặn)'.format(label)))
            length = None
        else:
            pts = [gm.to_world(*c) for c in path]
            length = sum(math.hypot(pts[n + 1][0] - pts[n][0], pts[n + 1][1] - pts[n][1])
                         for n in range(len(pts) - 1))
        legs.append({'goal': k, 'clearance': cl, 'length': length,
                     'path': None if path is None else [gm.to_world(*c) for c in path]})
        # camera
        pose = (g['x'], g['y'], g['yaw'])
        views[k] = []
        for o in objects:
            ok, dist, bearing, note = camera_view(gm, walls, pose, o, obj_masks[o['name']])
            views[k].append({'object': o['name'], 'coco': o.get('coco'), 'visible': ok,
                             'dist': dist, 'bearing': bearing, 'note': note})
        if objects and not any(v['visible'] for v in views[k]):
            issues.append(('GHI CHÚ', '{}: camera không nhìn thấy vật thể nào khi tới nơi '
                           '(vẫn đạt yêu cầu điều hướng; muốn YOLO nhận diện ở goal thì xoay yaw '
                           'về phía vật thể)'.format(label)))
        prev = pose
    return {'issues': issues, 'legs': legs, 'views': views, 'start': start, 'clearance': clear,
            'passable': passable, 'obj_masks': obj_masks}


def print_report(gm, init, goals, objects, res):
    print('Bản đồ   : {} ({} x {} ô, {:.3f} m/ô)'.format(gm.path, gm.w, gm.h, gm.res))
    print('Xuất phát: initial_pose ({:.2f}, {:.2f}, yaw {:.0f}°) -> sau undock ~({:.2f}, {:.2f})'.format(
        init[0], init[1], math.degrees(init[2]), res['start'][0], res['start'][1]))
    print('Vật thể  : {}'.format(', '.join(o['name'] for o in objects) if objects else '(không dùng)'))
    print()
    print('{:<4} {:>8} {:>8} {:>7} {:>10} {:>10}  {}'.format(
        '#', 'x', 'y', 'yaw°', 'cách tường', 'quãng đg', 'tên'))
    total = 0.0
    for k, g in enumerate(goals):
        leg = res['legs'][k]
        cl = '-' if leg['clearance'] is None else '{:.2f} m'.format(leg['clearance'])
        ln = 'KHÔNG TỚI' if leg['length'] is None else '{:.1f} m'.format(leg['length'])
        total += leg['length'] or 0.0
        print('{:<4} {:>8.2f} {:>8.2f} {:>7.0f} {:>10} {:>10}  {}'.format(
            'G{}'.format(k + 1), g['x'], g['y'], math.degrees(g['yaw']), cl, ln, g['name']))
    print('Tổng quãng đường ước lượng (A*, chưa tính đoạn vòng tránh của Nav2): {:.1f} m'.format(total))
    if objects:
        print('\nCamera tại mỗi goal (FOV 60°):')
        for k in range(len(goals)):
            seen = ['{} {:.1f} m {:+.0f}° ({})'.format(v['object'], v['dist'], math.degrees(v['bearing']),
                                                       v['note'])
                    for v in res['views'][k] if v['visible']]
            print('  G{}: {}'.format(k + 1, '; '.join(seen) if seen else 'không thấy vật thể nào'))
    print()
    if not res['issues']:
        print('KẾT LUẬN: các goal HỢP LỆ.')
    for level, msg in res['issues']:
        print('[{}] {}'.format(level, msg))


# ------------------------------------------------------------------------------------------- vẽ
CLASS_COLORS = {'person': '#e377c2', 'car': '#1f77b4', 'chair': '#8c564b', 'fire hydrant': '#ff7f0e',
                None: '#9467bd'}


def plot(gm, init, goals, objects, res, out, run=None, detections=None, title=None):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Polygon, Wedge

    img = np.full(gm.occ.shape + (3,), 0.80)
    img[gm.occ == 0] = 1.0
    img[gm.occ == 100] = 0.10
    infl = (res['clearance'] < 0.45) & (gm.occ == 0)
    img[infl] = img[infl] * 0.7 + np.array([0.75, 0.85, 1.0]) * 0.3
    fig, ax = plt.subplots(figsize=(9, 9))
    ax.imshow(img, origin='lower', extent=gm.extent(), interpolation='nearest')

    for o in objects:
        color = CLASS_COLORS.get(o.get('coco'), '#9467bd')
        pts = footprint_points(o, n=16)
        if (o.get('footprint') or {}).get('box') is not None:
            x0, x1, y0, y1 = o['footprint']['box']
            c, s = math.cos(o.get('yaw', 0.0)), math.sin(o.get('yaw', 0.0))
            corners = [(o['x'] + c * a - s * b, o['y'] + s * a + c * b)
                       for a, b in ((x0, y0), (x1, y0), (x1, y1), (x0, y1))]
            ax.add_patch(Polygon(corners, closed=True, color=color, alpha=0.6))
        else:
            ax.add_patch(Polygon(pts[1:], closed=True, color=color, alpha=0.8))
        ax.annotate(o['name'], (o['x'], o['y']), xytext=(4, 4), textcoords='offset points',
                    fontsize=7, color=color, weight='bold')

    for leg in res['legs']:
        if leg['path'] and run is None:
            xs, ys = zip(*leg['path'])
            ax.plot(xs, ys, '--', color='#2ca02c', lw=1.2, alpha=0.8)

    if run:
        for k, plan in enumerate(run.get('plans', [])):
            if plan:
                xs, ys = zip(*plan)
                ax.plot(xs, ys, '-', color='#2ca02c', lw=1.0, alpha=0.7,
                        label='Đường Nav2 lập (/plan)' if k == 0 else None)
        traj = run.get('trajectory') or []
        if traj:
            ax.plot([p[1] for p in traj], [p[2] for p in traj], '-', color='#d62728', lw=1.6,
                    label='Quỹ đạo robot (/amcl_pose)')

    if detections:
        seen = set()
        for d in detections:
            c = CLASS_COLORS.get(d['class'], '#9467bd')
            ax.plot(d['x'], d['y'], 'o', ms=3, color=c, alpha=0.5,
                    label='YOLO: {}'.format(d['class']) if d['class'] not in seen else None)
            seen.add(d['class'])

    sx, sy, syaw = init
    ax.plot(sx, sy, 's', color='k', ms=7, label='Xuất phát (dock)')
    for k, g in enumerate(goals):
        ax.add_patch(Wedge((g['x'], g['y']), 3.0, math.degrees(g['yaw'] - CAMERA_HFOV / 2),
                           math.degrees(g['yaw'] + CAMERA_HFOV / 2), color='#ffd700', alpha=0.25))
        ax.arrow(g['x'], g['y'], 0.6 * math.cos(g['yaw']), 0.6 * math.sin(g['yaw']), width=0.06,
                 color='#006400', length_includes_head=True)
        ax.plot(g['x'], g['y'], 'o', color='#006400', ms=9)
        ax.annotate('G{}'.format(k + 1), (g['x'], g['y']), xytext=(-14, 8), textcoords='offset points',
                    fontsize=11, weight='bold', color='#006400')
    ax.set_xlabel('x (m) – frame map')
    ax.set_ylabel('y (m)')
    ax.set_title(title or 'Bản đồ, goal và vật thể')
    ax.grid(True, alpha=0.25)
    handles, labels = ax.get_legend_handles_labels()
    if handles:
        ax.legend(loc='lower left', fontsize=7, framealpha=0.85)
    fig.tight_layout()
    fig.savefig(out, dpi=130)
    plt.close(fig)


def load_detections(path):
    out = []
    with open(path, newline='', encoding='utf-8') as f:
        for row in csv.DictReader(f):
            try:
                if row.get('robot_x') in (None, '', 'nan'):
                    continue
                out.append({'class': row['class'], 'x': float(row['robot_x']), 'y': float(row['robot_y'])})
            except (KeyError, ValueError):
                continue
    return out


# ------------------------------------------------------------------------------------------- main
def default_map():
    path, tried = lc.find_config('maps', 'map.yaml')
    return path, tried


def main(argv=None):
    ap = argparse.ArgumentParser(description='Kiểm tra goal/vật thể trên bản đồ, vẽ ảnh cho báo cáo.')
    ap.add_argument('--map', help='map.yaml (mặc định: maps/map.yaml của package)')
    ap.add_argument('--goals', help='goals.yaml (mặc định: config/goals.yaml)')
    ap.add_argument('--objects', help='objects.yaml (mặc định: config/objects.yaml)')
    ap.add_argument('--no-objects', action='store_true', help='bỏ qua vật thể')
    ap.add_argument('--min-clearance', type=float, default=0.45)
    ap.add_argument('--plot', nargs='?', const='', default=None, metavar='OUT.png',
                    help='vẽ ảnh (mặc định ~/tb4_results/goals_map.png)')
    ap.add_argument('--run', help='run.json do waypoint_navigator.py lưu (vẽ quỹ đạo + /plan)')
    ap.add_argument('--detections', help='detections.csv do yolo_detector.py lưu (vẽ vị trí robot lúc phát hiện)')
    args = ap.parse_args(argv)

    map_path = args.map
    if not map_path:
        map_path, tried = default_map()
        if not map_path:
            print('Không tìm thấy maps/map.yaml. Đã tìm:\n  ' + '\n  '.join(tried))
            return 1
    goals_path = args.goals or lc.find_config('config', 'goals.yaml')[0]
    objects_path = None if args.no_objects else (args.objects or lc.find_config('config', 'objects.yaml')[0])
    if not goals_path:
        print('Không tìm thấy config/goals.yaml')
        return 1
    try:
        gm = GridMap(map_path)
        init, goals = load_goals(goals_path)
        objects = load_objects(objects_path) if objects_path else []
    except (OSError, ValueError, KeyError, yaml.YAMLError) as exc:
        print('LỖI: {}'.format(exc))
        return 1
    if abs(gm.origin_yaw) > 1e-6:
        print('[CẢNH BÁO] origin của bản đồ có yaw khác 0 – script bỏ qua yaw này.')
    res = analyse(gm, init, goals, objects, min_clearance=args.min_clearance)
    print('Goal     : {}'.format(goals_path))
    print_report(gm, init, goals, objects, res)

    if args.plot is not None:
        out = args.plot or os.path.join(lc.results_dir(), 'goals_map.png')
        run = None
        if args.run:
            with open(args.run, 'r', encoding='utf-8') as f:
                run = json.load(f)
        dets = load_detections(args.detections) if args.detections else None
        try:
            plot(gm, init, goals, objects, res, out, run=run, detections=dets,
                 title='Kết quả chạy: đường Nav2 lập + quỹ đạo robot' if run else None)
            print('\nĐã vẽ: {}'.format(out))
        except ImportError:
            print('\nKhông vẽ được: thiếu matplotlib -> sudo apt install -y python3-matplotlib')
    return 1 if any(level == 'LỖI' for level, _ in res['issues']) else 0


if __name__ == '__main__':
    sys.exit(main())
