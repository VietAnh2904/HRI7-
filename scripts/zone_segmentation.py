#!/usr/bin/env python3
"""Bài 7 – Phân vùng bản đồ (Map Segmentation) + gắn nhãn khu vực (Semantic Mapping). KHÔNG cần ROS chạy.

  ros2 run tb4_lab04 zone_segmentation.py                  # maps/map.yaml -> maps/zones.yaml + ảnh
  ros2 run tb4_lab04 zone_segmentation.py --names none     # chỉ dùng ID A, B, C ... (bỏ tên có nghĩa)
  ros2 run tb4_lab04 zone_segmentation.py --manual config/zones_manual.yaml   # vùng vẽ tay (đa giác)
  ros2 run tb4_lab04 zone_segmentation.py --run ~/tb4_results/voice_<...>/voice_run.json  # + quỹ đạo

Đầu ra (mặc định cạnh map.yaml):
  maps/zones.yaml        tên, ID, tâm, điểm đích hợp lệ (x, y), diện tích, đa giác, khu vực kề nhau
  maps/zones_labels.pgm  ảnh nhãn cùng kích thước/gốc toạ độ với map.pgm: pixel = số thứ tự vùng + 1
                         (0 = không thuộc vùng nào) -> node điều hướng tra "robot đang ở khu vực nào"
  maps/map_zones.png     ảnh trực quan (bản đồ tô màu theo vùng + nhãn + điểm đích)

Thuật toán tự động (OpenCV + numpy):
  1. Lấy ô TRỐNG đã biết của occupancy grid (trinary: trống / vật cản / chưa biết), mở hình thái học
     (morphology opening) bỏ các đốm nhiễu 1–2 pixel của SLAM; chỉ giữ thành phần liên thông mà robot
     (bán kính 0.175 m) đi tới được từ điểm xuất phát -> mọi khu vực đều đến được.
  2. Distance transform: mỗi ô trống -> khoảng cách tới vật cản gần nhất. Tâm phòng = đỉnh cao,
     cửa / chỗ thắt của hành lang = "yên ngựa" thấp giữa hai đỉnh.
  3. Watershed theo độ bền (persistence): duyệt ô theo khoảng cách GIẢM dần (nước dâng từ đỉnh xuống).
     Hai lưu vực gặp nhau tại mức s: nếu chỗ gặp gần cao bằng đỉnh thấp hơn
     (s / đỉnh_thấp > merge_ratio hoặc đỉnh_thấp - s < persistence) thì đó KHÔNG phải cửa (chỉ là
     hành lang hơi gồ ghề) -> gộp; ngược lại giữ 2 vùng riêng, ranh giới chính là chỗ thắt (cửa).
  4. Vùng nhỏ hơn min_area gộp vào vùng kề có ranh giới chung dài nhất.
  5. Điểm đích của vùng: ô cách vật cản >= max(min_clearance, 80 % độ rộng vùng, tối đa 1 m), không trùng
     vật thể (config/objects.yaml), đi tới được, GẦN TÂM vùng nhất.
  6. Gắn tên: config/zone_names.yaml đặt tên có nghĩa cho vùng chứa một điểm neo (anchor); vùng còn lại
     tên "Khu vực <ID>". ID = A, B, C ... theo thứ tự đọc (trên -> dưới, trái -> phải).

Chế độ vẽ tay (--manual): mỗi vùng là một đa giác (toạ độ map, mét) trong file YAML – dùng khi muốn
kiểm soát ranh giới hoàn toàn; bước 1, 5, 6 vẫn áp dụng.

Chỉ dùng numpy + OpenCV + PyYAML (+ matplotlib để vẽ ảnh).
"""

import argparse
import heapq
import math
import os
import string
import sys

import numpy as np
import yaml

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
import lab_common as lc  # noqa: E402
from check_goals import GridMap, footprint_mask, load_objects, read_pgm  # noqa: E402

ROBOT_RADIUS = 0.175
DEFAULTS = {
    'merge_ratio': 0.70,      # s / đỉnh thấp > ngưỡng -> không phải cửa -> gộp
    'persistence': 0.30,      # (m) đỉnh thấp - s < ngưỡng -> gộp
    'min_peak': 0.35,         # (m) vùng có độ rộng nửa < ngưỡng (khe hẹp) luôn gộp vào vùng kề
    'min_area': 5.0,          # (m^2) vùng nhỏ hơn -> gộp vào vùng kề
    'min_clearance': 0.50,    # (m) điểm đích cách vật cản tối thiểu (inflation_radius của Nav2 = 0.45)
    'open_px': 3,             # kích thước kernel morphology opening (pixel)
}


# ------------------------------------------------------------------------------------------ tiện ích
def clearance_field(blocked, res):
    """Khoảng cách (m) từ mỗi ô tới ô bị chặn gần nhất (OpenCV distance transform, L2 chính xác)."""
    import cv2
    free = (~blocked).astype(np.uint8)
    return cv2.distanceTransform(free, cv2.DIST_L2, cv2.DIST_MASK_PRECISE) * res


def letter_id(k):
    """0 -> A, 25 -> Z, 26 -> AA ..."""
    letters = string.ascii_uppercase
    out = ''
    k += 1
    while k:
        k, r = divmod(k - 1, 26)
        out = letters[r] + out
    return out


def geodesic_from(passable, start, res):
    """Dijkstra 8 hướng trên các ô passable. Trả về mảng khoảng cách (m), inf = không tới được."""
    h, w = passable.shape
    dist = np.full((h, w), np.inf)
    si, sj = start
    if not (0 <= si < w and 0 <= sj < h) or not passable[sj, si]:
        return dist
    sq2 = math.sqrt(2.0) * res
    moves = [(1, 0, res), (-1, 0, res), (0, 1, res), (0, -1, res),
             (1, 1, sq2), (1, -1, sq2), (-1, 1, sq2), (-1, -1, sq2)]
    dist[sj, si] = 0.0
    heap = [(0.0, si, sj)]
    while heap:
        d, i, j = heapq.heappop(heap)
        if d > dist[j, i]:
            continue
        for di, dj, step in moves:
            ni, nj = i + di, j + dj
            if not (0 <= ni < w and 0 <= nj < h) or not passable[nj, ni]:
                continue
            if di and dj and not (passable[j, ni] and passable[nj, i]):
                continue
            nd = d + step
            if nd < dist[nj, ni]:
                dist[nj, ni] = nd
                heapq.heappush(heap, (nd, ni, nj))
    return dist


def nearest_true(mask, cell, max_r=40):
    """Ô True gần `cell` nhất (theo vòng vuông mở rộng) hoặc None."""
    i0, j0 = cell
    h, w = mask.shape
    for r in range(max_r + 1):
        best, bd = None, None
        for j in range(j0 - r, j0 + r + 1):
            for i in range(i0 - r, i0 + r + 1):
                if max(abs(i - i0), abs(j - j0)) != r:
                    continue
                if 0 <= i < w and 0 <= j < h and mask[j, i]:
                    d = (i - i0) ** 2 + (j - j0) ** 2
                    if bd is None or d < bd:
                        best, bd = (i, j), d
        if best is not None:
            return best
    return None


# ------------------------------------------------------------------------------------ bước 1: vùng trống
def navigable_free_space(gm, start_xy, robot_radius=ROBOT_RADIUS, open_px=3):
    """(free, reach): free = ô trống (đã lọc nhiễu) thuộc phần bản đồ robot tới được từ start_xy;
    reach = các ô mà TÂM robot đứng được (cách vật cản >= bán kính + 5 cm) và liên thông với start."""
    import cv2
    free = (gm.occ == 0).astype(np.uint8)
    if open_px and open_px > 1:
        free = cv2.morphologyEx(free, cv2.MORPH_OPEN, np.ones((open_px, open_px), np.uint8))
    clear = clearance_field(free == 0, gm.res)
    passable = (clear >= robot_radius + 0.05).astype(np.uint8)
    n, cc = cv2.connectedComponents(passable, connectivity=8)
    if n <= 1:
        raise ValueError('Bản đồ không có vùng trống nào đủ rộng cho robot')
    cell = nearest_true(passable.astype(bool), gm.to_cell(*start_xy), max_r=60)
    if cell is None:
        raise ValueError('Điểm xuất phát ({:.2f}, {:.2f}) không gần vùng trống nào'.format(*start_xy))
    reach = cc == cc[cell[1], cell[0]]
    # nới lại phần "lõi" robot tới được thêm đúng bán kính đã co -> toàn bộ sàn của các phòng tới được
    r_px = int(math.ceil((robot_radius + 0.05) / gm.res))
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * r_px + 1, 2 * r_px + 1))
    grown = cv2.dilate(reach.astype(np.uint8), kernel).astype(bool)
    return (free.astype(bool) & grown), reach


# --------------------------------------------------------------------------- bước 2–3: watershed + merge
def persistence_watershed(dist, mask, merge_ratio, persistence, min_peak):
    """Watershed trên -dist với gộp theo độ bền. Trả về mảng nhãn int32 (-1 ngoài mask, >= 0 vùng)."""
    h, w = dist.shape
    flat = np.where(mask, dist, -1.0).ravel()
    order = np.argsort(-flat, kind='stable')
    n_valid = int(np.count_nonzero(flat > 0))
    lab = np.full(h * w, -1, dtype=np.int64)
    parent = {}
    peak = {}

    def find(a):
        root = a
        while parent[root] != root:
            root = parent[root]
        while parent[a] != root:
            parent[a], a = root, parent[a]
        return root

    offsets = ((-1, -1), (0, -1), (1, -1), (-1, 0), (1, 0), (-1, 1), (0, 1), (1, 1))
    for idx in order[:n_valid]:
        idx = int(idx)
        j, i = divmod(idx, w)
        roots = set()
        best, best_d = -1, -1.0
        for di, dj in offsets:
            ni, nj = i + di, j + dj
            if 0 <= ni < w and 0 <= nj < h:
                k = nj * w + ni
                if lab[k] >= 0:
                    roots.add(find(lab[k]))
                    if flat[k] > best_d:
                        best, best_d = lab[k], flat[k]
        s = flat[idx]
        if not roots:                                   # đỉnh mới (cực đại địa phương)
            parent[idx] = idx
            peak[idx] = s
            lab[idx] = idx
            continue
        ordered = sorted(roots, key=lambda r: -peak[r])
        main = ordered[0]
        for r in ordered[1:]:
            a, b = find(r), find(main)
            if a == b:
                continue
            low = min(peak[a], peak[b])
            if low < min_peak or s / low > merge_ratio or low - s < persistence:
                hi, lo = (a, b) if peak[a] >= peak[b] else (b, a)
                parent[lo] = hi
                main = hi
        lab[idx] = best                                 # đi theo hướng dốc nhất -> ranh giới gọn
    out = np.full(h * w, -1, dtype=np.int32)
    valid = lab >= 0
    roots = np.array([find(int(v)) for v in lab[valid]], dtype=np.int64)
    uniq, inv = np.unique(roots, return_inverse=True)
    out[valid] = inv.astype(np.int32)
    return out.reshape(h, w), {int(k): float(peak[int(u)]) for k, u in enumerate(uniq)}


def adjacency(labels):
    """{(a, b): số cặp ô 8-kề nhau thuộc 2 vùng khác nhau}, a < b."""
    pairs = {}
    h, w = labels.shape
    for di, dj in ((1, 0), (0, 1), (1, 1), (1, -1)):
        j0, j1 = max(0, -dj), h - max(0, dj)
        i0, i1 = max(0, -di), w - max(0, di)
        a = labels[j0:j1, i0:i1]
        b = labels[j0 + dj:j1 + dj, i0 + di:i1 + di]
        m = (a >= 0) & (b >= 0) & (a != b)
        if not m.any():
            continue
        lo = np.minimum(a[m], b[m])
        hi = np.maximum(a[m], b[m])
        keys, counts = np.unique(np.stack([lo, hi], 1), axis=0, return_counts=True)
        for (x, y), c in zip(keys, counts):
            pairs[(int(x), int(y))] = pairs.get((int(x), int(y)), 0) + int(c)
    return pairs


def merge_small_regions(labels, min_cells):
    """Gộp lặp: vùng nhỏ nhất (< min_cells) vào vùng kề có ranh giới chung dài nhất. Đánh số lại 0..N-1."""
    labels = labels.copy()
    while True:
        ids, counts = np.unique(labels[labels >= 0], return_counts=True)
        if len(ids) <= 1:
            break
        small = [(c, k) for k, c in zip(ids, counts) if c < min_cells]
        if not small:
            break
        _, victim = min(small)
        adj = adjacency(labels)
        cands = [(n, b if a == victim else a) for (a, b), n in adj.items() if victim in (a, b)]
        if not cands:                     # vùng cô lập (không kề ai) – bỏ
            labels[labels == victim] = -1
            continue
        _, target = max(cands)
        labels[labels == victim] = target
    ids = np.unique(labels[labels >= 0])
    remap = np.full(int(labels.max()) + 2 if labels.size else 1, -1, dtype=np.int32)
    remap[ids] = np.arange(len(ids), dtype=np.int32)
    out = np.where(labels >= 0, remap[np.clip(labels, 0, None)], -1).astype(np.int32)
    return out


def rasterize_manual(gm, polygons_by_zone, free):
    """Chế độ vẽ tay: [{name, polygon:[[x,y],...]}] -> nhãn (giao với vùng trống)."""
    import cv2
    labels = np.full(gm.occ.shape, -1, dtype=np.int32)
    for k, z in enumerate(polygons_by_zone):
        pts = np.array([[(x - gm.ox) / gm.res, (y - gm.oy) / gm.res] for x, y in z['polygon']],
                       dtype=np.float64)
        canvas = np.zeros(gm.occ.shape, np.uint8)
        cv2.fillPoly(canvas, [np.round(pts).astype(np.int32)], 1)
        labels[(canvas > 0) & free & (labels < 0)] = k
    return labels


# --------------------------------------------------------------------------------- bước 5–6: mô tả vùng
def zone_polygons(gm, mask, eps_px=1.5, min_px=6):
    """Đường bao ngoài (toạ độ map) của từng mảnh của vùng (bỏ lỗ bên trong)."""
    import cv2
    contours, _ = cv2.findContours(mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    polys = []
    for c in sorted(contours, key=cv2.contourArea, reverse=True):
        if cv2.contourArea(c) < min_px:
            continue
        c = cv2.approxPolyDP(c, eps_px, True).reshape(-1, 2)
        polys.append([[round(gm.ox + (i + 0.5) * gm.res, 3), round(gm.oy + (j + 0.5) * gm.res, 3)]
                      for i, j in c])
    return polys


def choose_goal(gm, region, clear, reach, blocked_extra, min_clearance):
    """Điểm đích hợp lệ của vùng: (i, j, clearance) hoặc None."""
    ok = region & reach & ~blocked_extra
    if not ok.any():
        return None
    cmax = float(clear[ok].max())
    need = max(min(min_clearance, cmax), min(0.8 * cmax, 1.0))
    cand = ok & (clear >= need - 1e-9)
    jj, ii = np.nonzero(region)
    ci, cj = ii.mean(), jj.mean()
    cj_, ci_ = np.nonzero(cand)
    k = int(np.argmin((ci_ - ci) ** 2 + (cj_ - cj) ** 2))
    return int(ci_[k]), int(cj_[k]), float(clear[cj_[k], ci_[k]])


def load_names(path):
    """config/zone_names.yaml -> [{anchor:(x,y), name, aliases:[...]}]"""
    if not path or not os.path.isfile(path):
        return []
    with open(path, 'r', encoding='utf-8') as f:
        data = yaml.safe_load(f) or {}
    out = []
    for k, z in enumerate(data.get('zones') or []):
        if 'anchor' not in z or 'name' not in z:
            raise ValueError('{}: mục {} cần có anchor: [x, y] và name'.format(path, k + 1))
        ax, ay = [float(v) for v in z['anchor']]
        aliases = [str(a) for a in (z.get('aliases') or [])]
        out.append({'anchor': (ax, ay), 'name': str(z['name']), 'aliases': aliases,
                    'yaw': None if z.get('yaw') is None else float(z['yaw'])})
    return out


def load_manual(path):
    with open(path, 'r', encoding='utf-8') as f:
        data = yaml.safe_load(f) or {}
    zones = data.get('zones') or []
    for k, z in enumerate(zones):
        if len(z.get('polygon') or []) < 3:
            raise ValueError('{}: vùng {} cần polygon có >= 3 điểm'.format(path, k + 1))
    return zones


def segment(gm, start_xy=(0.0, 0.0), objects=(), names=(), manual=None, params=None):
    """Toàn bộ pipeline. Trả về dict {labels, zones[...], free, reach, clear, params}."""
    p = dict(DEFAULTS)
    p.update(params or {})
    free, reach = navigable_free_space(gm, start_xy, open_px=int(p['open_px']))
    clear = clearance_field(gm.occ != 0, gm.res)          # tới tường / vùng chưa biết của bản đồ gốc
    clear_free = clearance_field(~free, gm.res)

    if manual:
        labels = rasterize_manual(gm, manual, free)
        peaks = {}
    else:
        labels, peaks = persistence_watershed(clear_free, free, p['merge_ratio'], p['persistence'],
                                              p['min_peak'])
        labels = merge_small_regions(labels, int(round(p['min_area'] / gm.res ** 2)))

    blocked_extra = np.zeros(gm.occ.shape, bool)       # vật thể nới thêm bán kính robot + 25 cm
    obj_raw = np.zeros(gm.occ.shape, bool)
    for o in objects:
        blocked_extra |= footprint_mask(gm, o, margin=ROBOT_RADIUS + 0.25)
        obj_raw |= footprint_mask(gm, o)
    obj_clear = clearance_field(obj_raw, gm.res) if obj_raw.any() else None   # tới mép vật thể

    geo = geodesic_from(reach, nearest_true(reach, gm.to_cell(*start_xy), 60), gm.res)

    # thứ tự đọc: trên -> dưới (băng 3 m), trái -> phải
    raw = []
    n = int(labels.max()) + 1 if (labels >= 0).any() else 0
    for k in range(n):
        region = labels == k
        if not region.any():
            continue
        jj, ii = np.nonzero(region)
        cx = gm.ox + (ii.mean() + 0.5) * gm.res
        cy = gm.oy + (jj.mean() + 0.5) * gm.res
        raw.append((-math.floor(cy / 3.0), cx, k, region, cx, cy))
    raw.sort(key=lambda t: (t[0], t[1]))

    zones = []
    new_labels = np.full(labels.shape, -1, dtype=np.int32)
    for idx, (_, _, k, region, cx, cy) in enumerate(raw):
        new_labels[region] = idx
        g = choose_goal(gm, region, clear, reach, blocked_extra, p['min_clearance'])
        zid = letter_id(idx)
        z = {'id': zid, 'index': idx, 'name': 'Khu vực {}'.format(zid), 'aliases': [],
             'center': [round(cx, 3), round(cy, 3)],
             'area_m2': round(float(region.sum()) * gm.res ** 2, 2),
             'width_m': round(2.0 * float(clear_free[region].max()), 2)}
        if manual:
            z['name'] = str(manual[k].get('name') or z['name'])
            z['aliases'] = [str(a) for a in (manual[k].get('aliases') or [])]
        if g is None:
            z['goal'] = None
            z['valid'] = False
            z['issue'] = 'không có ô nào đủ rộng / tới được cho robot'
        else:
            gi, gj, gc = g
            gx, gy = gm.to_world(gi, gj)
            z['goal'] = {'x': round(gx, 3), 'y': round(gy, 3), 'yaw': None}
            z['goal_clearance_m'] = round(gc, 2)
            if obj_clear is not None:
                z['goal_object_distance_m'] = round(float(obj_clear[gj, gi]), 2)
            d = geo[gj, gi]
            z['path_from_start_m'] = None if not np.isfinite(d) else round(float(d), 2)
            z['valid'] = bool(np.isfinite(d) and gc >= ROBOT_RADIUS + 0.05)
            if not z['valid']:
                z['issue'] = 'không tới được từ điểm xuất phát' if not np.isfinite(d) else 'quá sát vật cản'
            elif gc < p['min_clearance']:
                z['issue'] = 'điểm đích chỉ cách vật cản {:.2f} m (< {:.2f} m)'.format(gc, p['min_clearance'])
        z['polygons'] = zone_polygons(gm, region)
        zones.append(z)

    # tên có nghĩa từ điểm neo
    used = {}
    for nm in names:
        i, j = gm.to_cell(*nm['anchor'])
        cell = (i, j) if gm.inside(i, j) and new_labels[j, i] >= 0 else \
            nearest_true(new_labels >= 0, (i, j), max_r=int(1.5 / gm.res))
        if cell is None:
            raise ValueError('Điểm neo "{}" ({:.2f}, {:.2f}) không nằm trong / gần khu vực nào'.format(
                nm['name'], *nm['anchor']))
        k = int(new_labels[cell[1], cell[0]])
        if k in used:
            raise ValueError('Hai tên "{}" và "{}" cùng rơi vào khu vực {} – sửa anchor trong '
                             'config/zone_names.yaml'.format(used[k], nm['name'], zones[k]['id']))
        used[k] = nm['name']
        zones[k]['name'] = nm['name']
        zones[k]['aliases'] = nm['aliases']
        if nm.get('yaw') is not None and zones[k]['goal']:
            zones[k]['goal']['yaw'] = nm['yaw']

    adj = adjacency(new_labels)
    for z in zones:
        z['neighbors'] = sorted(zones[b if a == z['index'] else a]['id']
                                for (a, b), c in adj.items() if z['index'] in (a, b) and c >= 4)
    return {'labels': new_labels, 'zones': zones, 'free': free, 'reach': reach, 'clear': clear,
            'params': p, 'peaks': peaks}


# ----------------------------------------------------------------------------------------- lưu / đọc
def plain(obj):
    """numpy -> kiểu Python thuần (để yaml.safe_dump ghi được)."""
    if isinstance(obj, dict):
        return {k: plain(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [plain(v) for v in obj]
    if isinstance(obj, np.generic):
        return obj.item()
    return obj


def write_label_pgm(path, labels):
    h, w = labels.shape
    if labels.max() >= 254:
        raise ValueError('Quá nhiều vùng (> 254)')
    img = (labels + 1).astype(np.uint8)[::-1]           # hàng 0 của ảnh = mép TRÊN (y lớn nhất)
    with open(path, 'wb') as f:
        f.write('P5\n# tb4_lab04 zones: pixel = index + 1, 0 = none\n{} {}\n255\n'.format(w, h).encode())
        f.write(img.tobytes())


def save_zones(path, gm, result, map_path, method, start_xy):
    out_dir = os.path.dirname(os.path.abspath(path))
    os.makedirs(out_dir, exist_ok=True)
    label_path = os.path.splitext(path)[0] + '_labels.pgm'
    write_label_pgm(label_path, result['labels'])
    zones = []
    for z in result['zones']:
        d = {k: z[k] for k in ('id', 'name', 'aliases', 'center', 'goal', 'valid', 'area_m2', 'width_m',
                               'neighbors') if k in z}
        for k in ('goal_clearance_m', 'goal_object_distance_m', 'path_from_start_m', 'issue'):
            if k in z:
                d[k] = z[k]
        d['polygons'] = z['polygons']
        zones.append(d)
    doc = {
        'map': os.path.relpath(os.path.abspath(map_path), out_dir),
        'labels_image': os.path.basename(label_path),
        'resolution': gm.res,
        'origin': [gm.ox, gm.oy, gm.origin_yaw],
        'frame_id': 'map',
        'method': method,
        'start': [float(start_xy[0]), float(start_xy[1])],
        'params': {k: float(v) for k, v in result['params'].items()},
        'zones': zones,
    }
    header = ('# Sinh tự động bởi scripts/zone_segmentation.py – đừng sửa tay toạ độ ở đây.\n'
              '# Đổi TÊN khu vực: sửa config/zone_names.yaml rồi chạy lại zone_segmentation.py.\n'
              '# goal.yaw = null -> voice_navigator tự chọn hướng = hướng di chuyển tới điểm đích.\n')
    with open(path, 'w', encoding='utf-8') as f:
        f.write(header)
        yaml.safe_dump(plain(doc), f, allow_unicode=True, sort_keys=False, default_flow_style=None, width=110)
    return label_path


class ZoneMap:
    """Đọc maps/zones.yaml (+ zones_labels.pgm) để tra cứu lúc chạy – không cần OpenCV."""

    def __init__(self, path):
        with open(path, 'r', encoding='utf-8') as f:
            doc = yaml.safe_load(f) or {}
        self.path = path
        self.zones = doc.get('zones') or []
        if not self.zones:
            raise ValueError('{}: không có khu vực nào'.format(path))
        self.res = float(doc['resolution'])
        self.ox, self.oy = float(doc['origin'][0]), float(doc['origin'][1])
        self.frame_id = doc.get('frame_id', 'map')
        self.labels = None
        lp = doc.get('labels_image')
        if lp:
            lp = lp if os.path.isabs(lp) else os.path.join(os.path.dirname(os.path.abspath(path)), lp)
            if os.path.isfile(lp):
                self.labels = read_pgm(lp)[::-1].astype(np.int32) - 1
        self.by_id = {z['id']: z for z in self.zones}

    def zone_at(self, x, y, snap=1.0):
        """ID khu vực chứa điểm (x, y); ngoài mọi vùng -> vùng gần nhất trong `snap` m; không có -> None."""
        if self.labels is not None:
            i = int(math.floor((x - self.ox) / self.res))
            j = int(math.floor((y - self.oy) / self.res))
            h, w = self.labels.shape
            if 0 <= i < w and 0 <= j < h and self.labels[j, i] >= 0:
                return self.zones[int(self.labels[j, i])]['id']
            if 0 <= i < w and 0 <= j < h:
                cell = nearest_true(self.labels >= 0, (i, j), max_r=int(snap / self.res))
                if cell is not None:
                    return self.zones[int(self.labels[cell[1], cell[0]])]['id']
            return None
        for z in self.zones:
            if any(point_in_polygon(x, y, poly) for poly in z.get('polygons') or []):
                return z['id']
        return None


def point_in_polygon(x, y, poly):
    inside = False
    n = len(poly)
    for k in range(n):
        x1, y1 = poly[k]
        x2, y2 = poly[(k + 1) % n]
        if (y1 > y) != (y2 > y):
            xc = x1 + (y - y1) * (x2 - x1) / (y2 - y1)
            if x < xc:
                inside = not inside
    return inside


# ------------------------------------------------------------------------------------------------- vẽ
PALETTE = ['#4e79a7', '#f28e2b', '#59a14f', '#e15759', '#76b7b2', '#edc948', '#b07aa1', '#ff9da7',
           '#9c755f', '#86bcb6', '#d37295', '#a0cbe8', '#8cd17d', '#f1ce63', '#bab0ac', '#fabfd2']


def plot_zones(gm, result, out, start_xy=None, title=None, route=None, run=None):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.colors import to_rgb

    labels = result['labels']
    img = np.full(gm.occ.shape + (3,), 0.80)
    img[gm.occ == 0] = 1.0
    img[gm.occ == 100] = 0.10
    for z in result['zones']:
        m = labels == z['index']
        img[m] = np.array(to_rgb(PALETTE[z['index'] % len(PALETTE)])) * 0.55 + 0.45
    fig, ax = plt.subplots(figsize=(9, 9))
    ax.imshow(img, origin='lower', extent=gm.extent(), interpolation='nearest')
    for z in result['zones']:
        color = PALETTE[z['index'] % len(PALETTE)]
        for poly in z['polygons']:
            xs, ys = zip(*(poly + poly[:1]))
            ax.plot(xs, ys, '-', color=color, lw=1.2)
        if z.get('goal'):
            gx, gy = z['goal']['x'], z['goal']['y']
            ax.plot(gx, gy, 'o', ms=9, color=color, mec='k', mew=1.2)
            label = z['id'] if z['name'] == 'Khu vực ' + z['id'] else '{}: {}'.format(z['id'], z['name'])
            ax.annotate(label, (gx, gy), xytext=(0, 9), textcoords='offset points', ha='center',
                        fontsize=10, weight='bold', color='k',
                        bbox=dict(boxstyle='round,pad=0.2', fc='white', ec=color, alpha=0.9))
    if route:
        xs, ys = zip(*route)
        ax.plot(xs, ys, '-', color='#d62728', lw=2.0, label='Đường đi dự kiến')
    if run:
        traj = run.get('trajectory') or []
        if traj:
            ax.plot([p[1] for p in traj], [p[2] for p in traj], '-', color='#d62728', lw=1.8,
                    label='Quỹ đạo robot (TF map->base_link)')
            ax.plot(traj[-1][1], traj[-1][2], 'v', color='#d62728', ms=9)
        from matplotlib.patches import Rectangle
        for k, (ox, oy, size) in enumerate(run.get('obstacles') or []):     # vật cản thêm lúc chạy (nếu ghi)
            ax.add_patch(Rectangle((ox - size / 2, oy - size / 2), size, size, color='k', zorder=5,
                                   label='Vật cản không có trên bản đồ' if k == 0 else None))
        cmds = [e for e in run.get('events', []) if e.get('event') == 'command']
        legs = [e for e in run.get('events', []) if e.get('event') == 'leg']
        ok = sum(1 for e in legs if e.get('status') == 'SUCCEEDED')
        title = title or '{} câu lệnh, {}/{} chặng tới đích'.format(len(cmds), ok, len(legs))
    if start_xy is not None:
        ax.plot(start_xy[0], start_xy[1], 's', color='k', ms=8, label='Xuất phát (dock)')
    ax.set_xlabel('x (m) – frame map')
    ax.set_ylabel('y (m)')
    ax.set_title(title or 'Phân vùng bản đồ: {} khu vực (chấm tròn = điểm đích Nav2)'.format(
        len(result['zones'])))
    ax.grid(True, alpha=0.25)
    ax.legend(loc='lower left', fontsize=8, framealpha=0.85)
    fig.tight_layout()
    fig.savefig(out, dpi=130)
    plt.close(fig)


# ----------------------------------------------------------------------------------------------- main
def print_table(result):
    print('{:<3} {:<26} {:>8} {:>8} {:>8} {:>7} {:>9} {:>9}  {}'.format(
        'ID', 'tên', 'goal x', 'goal y', 'd.tích', 'rộng', 'cách tg', 'từ dock', 'kề'))
    for z in result['zones']:
        g = z.get('goal') or {}
        print('{:<3} {:<26} {:>8} {:>8} {:>6.1f}m² {:>6.2f}m {:>9} {:>9}  {}'.format(
            z['id'], z['name'][:26],
            '-' if not g else '{:.2f}'.format(g['x']), '-' if not g else '{:.2f}'.format(g['y']),
            z['area_m2'], z['width_m'],
            '-' if 'goal_clearance_m' not in z else '{:.2f} m'.format(z['goal_clearance_m']),
            '-' if z.get('path_from_start_m') is None else '{:.1f} m'.format(z['path_from_start_m']),
            ','.join(z['neighbors'])))


def main(argv=None):
    ap = argparse.ArgumentParser(description='Phân vùng bản đồ occupancy grid thành các khu vực có nhãn.')
    ap.add_argument('--map', help='map.yaml (mặc định maps/map.yaml của package)')
    ap.add_argument('--out', help='zones.yaml đầu ra (mặc định cạnh map.yaml)')
    ap.add_argument('--names', help='file tên khu vực (mặc định config/zone_names.yaml); "none" = bỏ')
    ap.add_argument('--manual', help='file đa giác vẽ tay (bỏ qua watershed)')
    ap.add_argument('--objects', help='objects.yaml (mặc định config/objects.yaml) – điểm đích tránh vật thể')
    ap.add_argument('--no-objects', action='store_true')
    ap.add_argument('--start', nargs=2, type=float, default=[0.0, 0.0], metavar=('X', 'Y'),
                    help='điểm xuất phát của robot (frame map)')
    ap.add_argument('--plot', help='ảnh đầu ra (mặc định map_zones.png cạnh zones.yaml)')
    ap.add_argument('--run', help='voice_run.json do voice_navigator.py lưu -> vẽ thêm quỹ đạo robot')
    for k, v in DEFAULTS.items():
        ap.add_argument('--' + k.replace('_', '-'), type=type(v), default=v)
    args = ap.parse_args(argv)

    map_path = args.map or lc.find_config('maps', 'map.yaml')[0]
    if not map_path or not os.path.isfile(map_path):
        print('LỖI: không tìm thấy bản đồ {}'.format(map_path or 'maps/map.yaml'))
        return 1
    names_path = None if args.names == 'none' else (args.names or lc.find_config('config', 'zone_names.yaml')[0])
    objects_path = None if args.no_objects else (args.objects or lc.find_config('config', 'objects.yaml')[0])
    out = args.out or os.path.join(os.path.dirname(os.path.abspath(map_path)), 'zones.yaml')
    try:
        gm = GridMap(map_path)
        names = [] if args.manual else load_names(names_path)
        objects = load_objects(objects_path) if objects_path and os.path.isfile(objects_path) else []
        manual = load_manual(args.manual) if args.manual else None
        params = {k: getattr(args, k) for k in DEFAULTS}
        result = segment(gm, tuple(args.start), objects, names, manual, params)
    except (OSError, ValueError, KeyError, yaml.YAMLError) as exc:
        print('LỖI: {}'.format(exc))
        return 1
    if not result['zones']:
        print('LỖI: không tìm được khu vực nào')
        return 1
    method = 'manual:' + os.path.basename(args.manual) if args.manual else 'persistence_watershed'
    label_path = save_zones(out, gm, result, map_path, method, args.start)
    print('Bản đồ : {} ({} x {} ô, {:.3f} m/ô)'.format(map_path, gm.w, gm.h, gm.res))
    names_txt = 'theo file vẽ tay' if args.manual else (names_path or '(không)')
    print('Cách   : {} | tên: {} | vật thể: {}'.format(method, names_txt, objects_path or '(không)'))
    print('Kết quả: {} khu vực'.format(len(result['zones'])))
    print_table(result)
    bad = [z for z in result['zones'] if not z.get('valid')]
    for z in result['zones']:
        if z.get('issue'):
            print('[{}] {}: {}'.format('LỖI' if not z.get('valid') else 'CẢNH BÁO', z['id'], z['issue']))
    run = None
    if args.run:
        import json
        with open(args.run, 'r', encoding='utf-8') as f:
            run = json.load(f)
    if run is None:
        default_png = 'map_zones.png'
    else:
        default_png = os.path.join(os.path.dirname(os.path.abspath(args.run)), 'voice_run.png')
    plot_out = args.plot or os.path.join(os.path.dirname(out), default_png)
    try:
        plot_zones(gm, result, plot_out, start_xy=args.start, run=run)
        print('Đã lưu : {}\n         {}\n         {}'.format(out, label_path, plot_out))
    except ImportError:
        print('Đã lưu : {}\n         {}  (không vẽ ảnh: thiếu matplotlib)'.format(out, label_path))
    return 1 if bad else 0


if __name__ == '__main__':
    sys.exit(main())
