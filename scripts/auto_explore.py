#!/usr/bin/env python3
"""Tự khám phá và lập bản đồ (frontier exploration) cho TurtleBot 4 + SLAM Toolbox + Nav2.

Chạy cùng explore.launch.py (đã bật sẵn Gazebo, SLAM Toolbox, Nav2):

    ros2 launch tb4_lab04 explore.launch.py            # tự chạy node này
    ros2 run tb4_lab04 auto_explore.py                 # hoặc chạy riêng (explore:=false)

Thuật toán (Yamauchi 1997, "frontier-based exploration"):
  1. Đọc /map của SLAM Toolbox. "Frontier" = ô TRỐNG nằm cạnh ô CHƯA BIẾT.
  2. Gom các ô frontier liền nhau thành cụm; bỏ cụm nhỏ (nhiễu).
  3. Loang (wavefront) từ vị trí robot trên vùng trống đã nới vật cản theo bán kính robot
     -> biết cụm nào ĐI TỚI ĐƯỢC và quãng đường thật (không phải đường chim bay).
  4. Chọn cụm có chi phí nhỏ nhất (quãng đường - thưởng theo kích thước cụm). Chỗ đứng (goal) là ô
     robot đi tới được, cách tường >= goal_clearance và NHÌN THẲNG được tới frontier trong bán kính
     view_radius – không bắt robot chui sát vào góc tường (nơi Nav2 hay kẹt) mà vẫn quét được góc đó.
  5. Trong lúc đi: cụm đã được quét hết -> huỷ goal, chọn cụm khác; kẹt / quá giờ / Nav2 báo
     lỗi -> ghi nhận, thử lại tối đa 1 lần rồi bỏ qua vị trí đó.
  6. Hết frontier đi tới được (xác nhận nhiều lần) -> quay về điểm xuất phát (giúp SLAM khép
     vòng) -> lưu bản đồ bằng save_map.sh -> in tóm tắt.

Giới hạn (nói thẳng): chỉ "mọi ngóc ngách ĐI TỚI ĐƯỢC". Lối đi hẹp hơn đường kính robot + biên
an toàn, hoặc vùng Nav2 liên tục thất bại, sẽ bị bỏ qua và được liệt kê ở cuối. Vùng robot không
vào được nhưng LiDAR nhìn thấy vẫn có trên bản đồ.

Tự lưu: mỗi `autosave_period` giây mô phỏng lưu một bản vào <maps>/autosave/ (phòng khi mô
phỏng treo), và bản cuối vào <maps>/map.yaml (bản cũ được save_map.sh chuyển vào backup_*).

An toàn (thêm sau lỗi thực tế trên máy người dùng):
  * Trước khi lái: kiểm tra số đo LiDAR có hợp lý không (check_sensors.py). LiDAR "mù" (render đồ
    hoạ hỏng) -> DỪNG, giải thích, KHÔNG lái, KHÔNG ghi đè bản đồ.
  * Undock: kiểm tra robot THỰC SỰ đã lùi xa và quay mặt khỏi dock. Nếu phản xạ Create 3 cắt ngang
    undock (CANCELED) -> tự quay mặt khỏi dock và tiến ra; không được thì dừng và báo lỗi.
  * Điểm "về nhà" là chỗ robot đứng SAU khi đã ra khỏi dock (không phải chỗ dock).
  * Bản đồ gần như trống -> không coi là "khám phá xong", không lưu đè.
  * Goal thất bại / không tiến -> tự gỡ kẹt: lùi ra (cần safety_override của Create 3, explore.launch.py
    đã đặt), quay về hướng thoáng nhất theo LiDAR, tiến ra một đoạn ngắn rồi mới chọn goal mới.
"""

import math
import os
import signal
import subprocess
import sys
import time

try:
    import numpy as np
except KeyboardInterrupt:     # Ctrl+C đúng lúc node vừa khởi động
    sys.exit(0)

# ===================================================================================
# Phần logic thuần (không cần ROS) – có unit test trong test/test_pure_logic.py
# ===================================================================================

OCC_THRESH = 65     # giá trị OccupancyGrid >= ngưỡng này: vật cản
FREE_THRESH = 25    # 0 <= giá trị < ngưỡng này: trống; -1: chưa biết


def shift(mask, dy, dx):
    """Dịch mảng 2D (không cuộn vòng), ô trống điền False/0."""
    out = np.zeros_like(mask)
    h, w = mask.shape
    ys, yd = (slice(0, h - dy), slice(dy, h)) if dy >= 0 else (slice(-dy, h), slice(0, h + dy))
    xs, xd = (slice(0, w - dx), slice(dx, w)) if dx >= 0 else (slice(-dx, w), slice(0, w + dx))
    out[yd, xd] = mask[ys, xs]
    return out


def dilate(mask, radius_cells):
    """Nới vùng True theo hình tròn bán kính radius_cells."""
    if radius_cells <= 0:
        return mask.copy()
    out = mask.copy()
    r2 = radius_cells * radius_cells
    for dy in range(-radius_cells, radius_cells + 1):
        for dx in range(-radius_cells, radius_cells + 1):
            if (dy or dx) and dy * dy + dx * dx <= r2:
                out |= shift(mask, dy, dx)
    return out


def classify(grid):
    unknown = grid < 0
    occupied = grid >= OCC_THRESH
    free = (grid >= 0) & (grid < FREE_THRESH)
    return free, occupied, unknown


def erode(mask, radius_cells):
    return ~dilate(~mask, radius_cells)


def open_free(free):
    """Bỏ các vệt trống rộng 1-2 ô (tia LiDAR lọt vào trong khối tường) – chúng không phải lối
    đi mà chỉ tạo frontier giả bên trong vật cản."""
    return dilate(erode(free, 1), 1) & free


def frontier_mask(free, unknown):
    """Ô trống có ít nhất một ô lân cận (4 hướng) chưa biết."""
    near_unknown = (shift(unknown, 1, 0) | shift(unknown, -1, 0) |
                    shift(unknown, 0, 1) | shift(unknown, 0, -1))
    return free & near_unknown


def clusters(mask):
    """Gom ô True liền nhau (8 hướng). Trả về list mảng (N, 2) toạ độ (hàng, cột)."""
    cells = set(map(tuple, np.argwhere(mask)))
    out = []
    while cells:
        seed = cells.pop()
        stack, comp = [seed], [seed]
        while stack:
            r, c = stack.pop()
            for dr in (-1, 0, 1):
                for dc in (-1, 0, 1):
                    n = (r + dr, c + dc)
                    if n in cells:
                        cells.remove(n)
                        stack.append(n)
                        comp.append(n)
        out.append(np.array(comp, dtype=np.int32))
    return out


def wavefront(traversable, start):
    """Số bước (8 hướng) từ start tới mọi ô đi được; -1 = không tới được."""
    dist = np.full(traversable.shape, -1, dtype=np.int32)
    if not traversable[start]:
        return dist
    front = np.zeros(traversable.shape, dtype=bool)
    front[start] = True
    visited = front.copy()
    dist[start] = 0
    d = 0
    while front.any():
        d += 1
        grown = front.copy()
        for dy, dx in ((1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)):
            grown |= shift(front, dy, dx)
        new = grown & traversable & ~visited
        dist[new] = d
        visited |= new
        front = new
    return dist


def nearest_true(mask, cell, max_r):
    """Ô True gần `cell` nhất trong bán kính max_r (ô), hoặc None."""
    r0, c0 = cell
    h, w = mask.shape
    best, best_d = None, None
    for r in range(max(0, r0 - max_r), min(h, r0 + max_r + 1)):
        for c in range(max(0, c0 - max_r), min(w, c0 + max_r + 1)):
            if mask[r, c]:
                d = (r - r0) ** 2 + (c - c0) ** 2
                if best_d is None or d < best_d:
                    best, best_d = (r, c), d
    return best


class Candidate:
    def __init__(self, goal_cell, frontier_cell, size, path_cells, centroid, cells=None):
        self.goal_cell = goal_cell            # (hàng, cột) ô robot sẽ tới
        self.frontier_cell = frontier_cell    # ô frontier gần goal nhất (để biết cụm còn không)
        self.size = size                      # số ô frontier của cụm
        self.path_cells = path_cells          # quãng đường (số bước) từ robot
        self.centroid = centroid              # (hàng, cột) trọng tâm cụm
        self.cells = cells                    # mảng (N, 2) các ô của cụm


def visible_from(free, sources, max_cells, n_rays=72, unknown=None, gain_cells=3):
    """Các ô TRỐNG nhìn thấy được (tia thẳng chỉ đi qua ô trống) từ ít nhất một ô trong `sources`, trong
    bán kính max_cells. Trả về (mask, khoảng_cách_ngắn_nhất_tới_nguồn [ô], inf nếu không thấy).

    Dùng để chọn chỗ đứng: robot không cần chui sát vào frontier ở góc tường – chỉ cần đứng ở chỗ
    thoáng mà LiDAR "nhìn" thẳng được tới frontier đó.
    unknown (mảng bool): chỉ giữ hướng nhìn mà tia từ chỗ đứng XUYÊN QUA frontier còn đi tiếp vào ô chưa
    biết trong gain_cells ô – loại các chỗ đứng thẳng hàng với "bóng" do góc tường che (đứng đó nhìn thấy
    frontier nhưng không thấy gì phía sau nó)."""
    h, w = free.shape
    mask = np.zeros((h, w), dtype=bool)
    dmin = np.full((h, w), np.inf)
    if max_cells <= 0 or len(sources) == 0:
        return mask, dmin
    ang = np.linspace(-math.pi, math.pi, n_rays, endpoint=False)
    t = np.arange(0.0, max_cells + 0.25, 0.5)
    s, c = np.sin(ang)[:, None], np.cos(ang)[:, None]
    back = np.arange(0.5, gain_cells + 0.25, 0.5)[None, :]
    for r0, c0 in sources:
        rr = np.rint(r0 + s * t).astype(np.int64)
        cc = np.rint(c0 + c * t).astype(np.int64)
        inb = (rr >= 0) & (rr < h) & (cc >= 0) & (cc < w)
        ok = inb & free[np.clip(rr, 0, h - 1), np.clip(cc, 0, w - 1)]
        first_bad = np.where(ok.all(axis=1), ok.shape[1], np.argmax(~ok, axis=1))
        valid = np.arange(ok.shape[1])[None, :] < first_bad[:, None]
        if unknown is not None:
            # phía bên kia frontier theo hướng nhìn (ngược hướng tia) phải có ô chưa biết
            br = np.rint(r0 - s * back).astype(np.int64)
            bc = np.rint(c0 - c * back).astype(np.int64)
            binb = (br >= 0) & (br < h) & (bc >= 0) & (bc < w)
            bu = binb & unknown[np.clip(br, 0, h - 1), np.clip(bc, 0, w - 1)]
            bfree = binb & free[np.clip(br, 0, h - 1), np.clip(bc, 0, w - 1)]
            # có ô chưa biết trước khi gặp vật cản/ra ngoài bản đồ
            stop = ~(bu | bfree)
            first_stop = np.where(stop.any(axis=1), np.argmax(stop, axis=1), stop.shape[1])
            first_unk = np.where(bu.any(axis=1), np.argmax(bu, axis=1), bu.shape[1] + 1)
            valid &= (first_unk < first_stop)[:, None]
        vr, vc = rr[valid], cc[valid]
        mask[vr, vc] = True
        np.minimum.at(dmin, (vr, vc), np.broadcast_to(t, valid.shape)[valid])
    return mask, dmin


def find_candidates(grid, robot_cell, resolution, robot_radius=0.22, min_frontier_cells=10,
                    approach_radius=0.35, blocked=None, view_radius=0.0, clearance=0.0,
                    frontier_weight=0.5, max_sources=16, avoid=None):
    """Trả về (danh_sách_Candidate, số_cụm_frontier_tổng, ô_bắt_đầu_loang).

    grid: mảng int (hàng = y, cột = x) theo đúng OccupancyGrid.
    blocked: mảng bool cùng kích thước – các ô frontier "chết" (đã tới sát mà không quét được).
    view_radius > 0: ngoài các ô cách frontier <= approach_radius, nhận cả các ô robot tới được mà
        NHÌN THẲNG được tới frontier trong bán kính view_radius (m).
    clearance > 0: ưu tiên ô cách vật cản >= clearance (m) – tránh đẩy robot vào sát tường/góc (nơi
        Nav2 hay kẹt); chỉ khi cụm không có ô nào như vậy mới dùng ô sát tường.
    Goal = ô có chi phí nhỏ nhất: quãng đường từ robot + frontier_weight * khoảng cách tới frontier.
    avoid: mảng bool – các ô KHÔNG được chọn làm chỗ đứng (chỗ đã đứng mà không quét thêm được). Cụm
        frontier vẫn được giữ và lấy chỗ đứng tốt nhất còn lại – thường là nhìn từ hướng khác (vd. vòng
        sang phía bên kia khối tường để thấy mặt bị che), thay vì bỏ cả cụm.
    """
    free, occupied, unknown = classify(grid)
    r_cells = max(1, int(math.ceil(robot_radius / resolution)))
    traversable = free & ~dilate(occupied, r_cells)
    start = robot_cell
    h, w = grid.shape
    inside = 0 <= start[0] < h and 0 <= start[1] < w
    if not inside or not traversable[start]:
        # Robot đứng sát tường (trong vùng nới) -> bắt đầu loang từ ô đi được gần nhất.
        start = nearest_true(traversable, start, int(0.6 / resolution)) if inside else None
    if start is None:
        return [], 0, None
    dist = wavefront(traversable, start)
    reach = dist >= 0
    c_cells = int(math.ceil(clearance / resolution)) if clearance > 0 else 0
    comfy = free & ~dilate(occupied, c_cells) if c_cells > r_cells else traversable

    # Vùng trống LIỀN với robot mà không phải chui qua khe 1-2 ô trong tường: tia LiDAR lọt qua
    # khe ở góc tường tạo ra vùng "trống" bên trong khối tường – frontier ở đó không bao giờ quét
    # được. Nới tường 1 ô sẽ bịt các khe này nhưng không ảnh hưởng lối đi thật (>= 0.45 m).
    free_safe = free & ~dilate(occupied, 1)
    s2 = start if free_safe[start] else nearest_true(free_safe, start, int(0.6 / resolution))
    connected = dilate(wavefront(free_safe, s2) >= 0, 2) if s2 is not None else reach

    fmask = frontier_mask(open_free(free), unknown)
    if blocked is not None:
        fmask &= ~blocked
    groups = clusters(fmask)
    k = max(1, int(math.ceil(approach_radius / resolution)))
    kv = int(math.ceil(view_radius / resolution)) if view_radius > approach_radius else 0
    pad = max(k, kv)
    out = []
    for g in groups:
        if len(g) < min_frontier_cells:
            continue
        if not connected[g[:, 0], g[:, 1]].any():
            continue
        r_lo, c_lo = np.maximum(g.min(axis=0) - pad, 0)
        r_hi, c_hi = np.minimum(g.max(axis=0) + pad + 1, (h, w))
        local = np.zeros((r_hi - r_lo, c_hi - c_lo), dtype=bool)
        local[g[:, 0] - r_lo, g[:, 1] - c_lo] = True
        reach_l = reach[r_lo:r_hi, c_lo:c_hi]
        near = dilate(local, k) & reach_l
        if kv:
            step = max(1, len(g) // max_sources)
            src = [(int(r) - r_lo, int(q) - c_lo) for r, q in g[::step]]
            vis, vdist = visible_from(free[r_lo:r_hi, c_lo:c_hi], src, kv,
                                      unknown=unknown[r_lo:r_hi, c_lo:c_hi])
            near = near | (vis & reach_l)
        if avoid is not None:
            near &= ~avoid[r_lo:r_hi, c_lo:c_hi]
        if not near.any():
            continue
        pool = near & comfy[r_lo:r_hi, c_lo:c_hi]
        if not pool.any():
            pool = near
        # khoảng cách (ô) từ mỗi ô trong pool tới ô frontier gần nhất của cụm
        pr, pc = np.nonzero(pool)
        gs = g[::max(1, len(g) // 200)]
        d_front = np.sqrt(((pr[:, None] + r_lo - gs[None, :, 0]) ** 2 +
                           (pc[:, None] + c_lo - gs[None, :, 1]) ** 2).min(axis=1))
        d_path = dist[r_lo:r_hi, c_lo:c_hi][pr, pc].astype(float)
        i = int(np.argmin(d_path + frontier_weight * d_front))
        goal = (int(pr[i] + r_lo), int(pc[i] + c_lo))
        d2 = (g[:, 0] - goal[0]) ** 2 + (g[:, 1] - goal[1]) ** 2
        fcell = tuple(int(v) for v in g[int(np.argmin(d2))])
        centroid = tuple(float(v) for v in g.mean(axis=0))
        out.append(Candidate(goal, fcell, len(g), int(dist[goal]), centroid, g))
    return out, len(groups), start


def stamp_discs(shape, cells, radius_cells):
    """Mảng bool với các hình tròn bán kính radius_cells quanh từng ô trong `cells`."""
    out = np.zeros(shape, dtype=bool)
    h, w = shape
    r2 = radius_cells * radius_cells
    for r0, c0 in cells:
        r_lo, r_hi = max(0, r0 - radius_cells), min(h, r0 + radius_cells + 1)
        c_lo, c_hi = max(0, c0 - radius_cells), min(w, c0 + radius_cells + 1)
        if r_lo >= r_hi or c_lo >= c_hi:
            continue
        yy, xx = np.ogrid[r_lo:r_hi, c_lo:c_hi]
        out[r_lo:r_hi, c_lo:c_hi] |= (yy - r0) ** 2 + (xx - c0) ** 2 <= r2
    return out


def score(c, resolution, gain_weight=1.0, max_gain_m=4.0):
    """Chi phí: quãng đường (m) trừ một phần thưởng theo kích thước cụm (m). Nhỏ hơn = tốt hơn."""
    return c.path_cells * resolution - gain_weight * min(c.size * resolution, max_gain_m)


def place_key(x, y, cell=0.5):
    """Khoá rời rạc hoá vị trí để đếm số lần thử tại cùng một chỗ."""
    return (int(math.floor(x / cell)), int(math.floor(y / cell)))


def frontier_still_there(grid, cell, radius_cells):
    """Còn ô frontier nào trong bán kính radius_cells quanh `cell` không."""
    free, _, unknown = classify(grid)
    free = open_free(free)
    h, w = grid.shape
    r0, c0 = cell
    r_lo, r_hi = max(0, r0 - radius_cells - 1), min(h, r0 + radius_cells + 2)
    c_lo, c_hi = max(0, c0 - radius_cells - 1), min(w, c0 + radius_cells + 2)
    f = frontier_mask(free[r_lo:r_hi, c_lo:c_hi], unknown[r_lo:r_hi, c_lo:c_hi])
    pts = np.argwhere(f)
    if len(pts) == 0:
        return False
    d2 = (pts[:, 0] + r_lo - r0) ** 2 + (pts[:, 1] + c_lo - c0) ** 2
    return bool((d2 <= radius_cells * radius_cells).any())


def known_cells_around(grid, cell, radius_cells):
    """Số ô ĐÃ BIẾT (trống hoặc vật cản) trong hình tròn bán kính radius_cells quanh `cell`."""
    h, w = grid.shape
    r0, c0 = cell
    r_lo, r_hi = max(0, r0 - radius_cells), min(h, r0 + radius_cells + 1)
    c_lo, c_hi = max(0, c0 - radius_cells), min(w, c0 + radius_cells + 1)
    if r_lo >= r_hi or c_lo >= c_hi:
        return 0
    yy, xx = np.ogrid[r_lo:r_hi, c_lo:c_hi]
    disk = (yy - r0) ** 2 + (xx - c0) ** 2 <= radius_cells * radius_cells
    return int(((grid[r_lo:r_hi, c_lo:c_hi] >= 0) & disk).sum())


DEFAULT_DOCK_POSE = (0.157, 0.0, math.pi)   # odom: dock đặt trước điểm spawn 0.157 m, quay về robot
DOCK_BODY_OFFSET = -0.045                     # tâm thân dock trong frame std_dock_link (urdf)


def wrap_angle(a):
    return math.atan2(math.sin(a), math.cos(a))


def dock_body_xy(dock_pose):
    """Tâm phần thân đứng của dock (x, y) từ pose frame std_dock_link (x, y, yaw)."""
    x, y, yaw = dock_pose
    return x + DOCK_BODY_OFFSET * math.cos(yaw), y + DOCK_BODY_OFFSET * math.sin(yaw)


def away_heading(robot_xy, dock_xy):
    """Hướng (yaw) nhìn từ dock ra phía robot = hướng robot cần quay mặt để đi ra xa dock."""
    return math.atan2(robot_xy[1] - dock_xy[1], robot_xy[0] - dock_xy[0])


def undock_state(robot, dock_xy, min_dist=0.45, max_heading_err=math.radians(60.0)):
    """robot = (x, y, yaw) trong frame odom. Trả về (đã_rời_dock_đúng_cách, khoảng_cách, sai_lệch_hướng).

    Undock chuẩn của Create 3: lùi 0.4 m rồi quay 180 độ -> robot cách thân dock ~0.55 m và quay
    mặt RA XA dock. Nếu undock bị phản xạ cắt ngang, robot chỉ nhích vài cm và vẫn quay mặt vào dock.
    """
    d = math.hypot(robot[0] - dock_xy[0], robot[1] - dock_xy[1])
    err = abs(wrap_angle(robot[2] - away_heading(robot[:2], dock_xy)))
    return (d >= min_dist and err <= max_heading_err), d, err


def front_free_distance(points, half_width=0.2, max_x=5.0):
    """Khoảng trống phía trước (m) từ các điểm scan trong frame base_link: x nhỏ nhất của điểm nằm
    trong hành lang |y| <= half_width, 0 < x <= max_x. Không có điểm nào -> max_x."""
    best = max_x
    for x, y in points:
        if 0.0 < x <= best and abs(y) <= half_width:
            best = x
    return best


def rear_free_distance(points, half_width=0.2, max_x=5.0):
    """Khoảng trống phía SAU (m): -x nhỏ nhất của điểm có x < 0 và |y| <= half_width."""
    best = max_x
    for x, y in points:
        if -best <= x < 0.0 and abs(y) <= half_width:
            best = -x
    return best


def nearest_obstacle(points, min_range=0.05):
    """(hướng rad, khoảng cách m) của điểm scan gần tâm robot nhất; không có điểm -> None."""
    best = None
    for x, y in points:
        r = math.hypot(x, y)
        if r >= min_range and (best is None or r < best[1]):
            best = (math.atan2(y, x), r)
    return best


def best_open_heading(points, sector=math.radians(30.0), step=math.radians(10.0), max_range=4.0,
                      min_range=0.05, avoid=None):
    """Hướng (rad, so với mặt robot, trong [-pi, pi)) thoáng nhất: với mỗi hướng ứng viên, độ thoáng
    = khoảng cách điểm gần nhất nằm trong nón ±sector/2; chọn hướng thoáng nhất, hoà thì ưu tiên hướng
    gần phía trước (ít phải quay). avoid=(hướng, nửa_góc): bỏ các hướng ứng viên lệch < nửa_góc so với
    `hướng` (vd. phía bức tường robot đang tì vào). Trả về (hướng, độ_thoáng)."""
    polar = []
    for x, y in points:
        r = math.hypot(x, y)
        if min_range <= r <= max_range:
            polar.append((math.atan2(y, x), r))
    best_h, best_c = 0.0, -1.0
    n = int(round(2.0 * math.pi / step))
    for i in range(n):
        h = -math.pi + i * step
        if avoid is not None and abs(math.atan2(math.sin(h - avoid[0]), math.cos(h - avoid[0]))) < avoid[1]:
            continue
        c = max_range
        for a, r in polar:
            if abs(math.atan2(math.sin(a - h), math.cos(a - h))) <= sector / 2.0 and r < c:
                c = r
        if c > best_c + 1e-6 or (abs(c - best_c) <= 1e-6 and abs(h) < abs(best_h)):
            best_h, best_c = h, c
    return best_h, best_c


def scan_to_base(scan, lidar_pose):
    """Điểm (x, y) trong base_link từ LaserScan và pose LiDAR (x, y, yaw) trong base_link."""
    tx, ty, tyaw = lidar_pose
    c, s = math.cos(tyaw), math.sin(tyaw)
    pts = []
    for i, r in enumerate(scan.ranges):
        if not (math.isfinite(r) and scan.range_min <= r <= scan.range_max):
            continue
        a = scan.angle_min + i * scan.angle_increment
        lx, ly = r * math.cos(a), r * math.sin(a)
        pts.append((tx + c * lx - s * ly, ty + s * lx + c * ly))
    return pts


def resolve_maps_dir(pkg_share, fallback):
    """<thư mục source>/maps (giống save_map.sh), nếu không có thì fallback."""
    try:
        with open(os.path.join(pkg_share, 'source_dir.txt'), 'r', encoding='utf-8') as f:
            src = f.read().strip()
        if src and os.path.isdir(src) and os.access(src, os.W_OK):
            return os.path.join(src, 'maps')
    except OSError:
        pass
    return fallback


# ===================================================================================
# Phần ROS
# ===================================================================================

def main():  # noqa: C901  (vòng lặp điều khiển dài nhưng tuần tự, dễ đọc hơn khi để chung)
    import rclpy
    from action_msgs.msg import GoalStatus
    from ament_index_python.packages import get_package_prefix, get_package_share_directory
    from geometry_msgs.msg import Twist
    from irobot_create_msgs.action import Undock
    from irobot_create_msgs.msg import DockStatus, HazardDetectionVector
    from nav2_msgs.action import NavigateToPose
    from nav_msgs.msg import OccupancyGrid
    from rclpy.action import ActionClient
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy,
                           qos_profile_sensor_data)
    from rclpy.time import Time
    from sensor_msgs.msg import LaserScan
    from tf2_ros import TransformException
    from tf2_ros.buffer import Buffer
    from tf2_ros.transform_listener import TransformListener

    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from check_sensors import (HAZARD_NAMES, REFLEX_HAZARDS, RENDER_FIX, SCAN_ADVICE,
                               scan_health)

    rclpy.init(args=sys.argv)
    node = Node('tb4_lab04_auto_explore',
                parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])
    P = {}
    for name, default in [
        ('robot_radius', 0.22),          # bán kính robot + biên dùng để xét lối đi được
        ('min_frontier_size', 0.3),      # m – cụm frontier ngắn hơn coi là nhiễu
        ('view_radius', 1.5),            # m – được đứng xa frontier tới mức này nếu LiDAR nhìn thẳng tới
        ('goal_clearance', 0.4),         # m – ưu tiên chỗ đứng cách tường >= mức này (tránh kẹt góc)
        ('max_stuck_escapes', 3),
        ('max_views_per_frontier', 3),   # số chỗ đứng KHÁC NHAU thử cho 1 đoạn biên trước khi bỏ qua nó        # số lần tự gỡ kẹt liên tiếp không nhúc nhích thì dừng + lưu
        ('goal_timeout', 240.0),         # s mô phỏng cho mỗi goal
        ('no_progress_timeout', 30.0),   # s mô phỏng không tiến thêm 0.2 m -> coi là kẹt
        ('replan_period', 5.0),          # s mô phỏng giữa 2 lần đánh giá lại
        ('max_duration', 3600.0),        # s mô phỏng tối đa cho cả quá trình
        ('done_confirmations', 3),       # số lần liên tiếp không còn frontier mới kết thúc
        ('return_home', True),
        ('undock', True),
        ('undock_settle', 12.0),         # s mô phỏng chờ phản xạ Create 3 sau undock
        ('save_map', True),
        ('map_dir', ''),                 # '' = <source>/maps như save_map.sh
        ('autosave_period', 300.0),      # s mô phỏng; 0 = tắt
        ('startup_timeout', 600.0),      # s thực chờ /map, TF, Nav2 (máy chậm: Nav2 lên lâu)
        ('check_sensors', True),         # kiểm tra LiDAR hợp lý trước khi lái
        ('escape_distance', 0.55),       # m – khoảng cách tối thiểu từ thân dock sau khi undock
        ('min_explored_area', 4.0),      # m² – bản đồ nhỏ hơn thì chưa được kết luận "xong"
        ('min_save_area', 5.0),          # m² vùng trống tối thiểu để lưu bản đồ cuối
        # Ctrl+C -> thử lưu bản đồ: đúng khi `ros2 run` (SLAM ở terminal khác còn sống). explore.launch.py
        # đặt False: Ctrl+C tắt luôn SLAM nên lưu chắc chắn hỏng, chỉ làm launch phải SIGTERM script.
        ('save_on_ctrl_c', True),
    ]:
        node.declare_parameter(name, default)
        P[name] = node.get_parameter(name).value
    log = node.get_logger()

    state = {'map': None, 'dock': None, 'scan': None, 'hz': set()}

    def on_hz(m):
        for d in m.detections:
            name = HAZARD_NAMES.get(d.type, str(d.type))
            if name in REFLEX_HAZARDS:
                state['hz'].add(name)
    map_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(OccupancyGrid, '/map', lambda m: state.__setitem__('map', m), map_qos)
    node.create_subscription(DockStatus, '/dock_status',
                             lambda m: state.__setitem__('dock', m), qos_profile_sensor_data)
    node.create_subscription(LaserScan, '/scan', lambda m: state.__setitem__('scan', m),
                             qos_profile_sensor_data)
    node.create_subscription(HazardDetectionVector, '/hazard_detection', on_hz,
                             qos_profile_sensor_data)
    cmd_pub = node.create_publisher(Twist, '/cmd_vel', 10)
    tf_buffer = Buffer()
    TransformListener(tf_buffer, node)
    nav = ActionClient(node, NavigateToPose, 'navigate_to_pose')

    def sim_now():
        return node.get_clock().now().nanoseconds * 1e-9

    def spin_for(sec):
        end = time.monotonic() + sec
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=min(0.05, max(0.0, end - time.monotonic())))

    def sim_sleep(sec, wall_cap=None):
        start = sim_now()
        end = time.monotonic() + (wall_cap if wall_cap else max(30.0, sec * 20.0))
        while rclpy.ok() and sim_now() - start < sec and time.monotonic() < end:
            spin_for(0.1)

    def lookup_2d(target, source):
        try:
            t = tf_buffer.lookup_transform(target, source, Time())
        except TransformException:
            return None
        tr, q = t.transform.translation, t.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return tr.x, tr.y, yaw

    def robot_pose():
        return lookup_2d('map', 'base_link')

    def grid_of(m):
        return np.asarray(m.data, dtype=np.int16).reshape(m.info.height, m.info.width)

    def world_to_cell(m, x, y):
        res = m.info.resolution
        return (int(math.floor((y - m.info.origin.position.y) / res)),
                int(math.floor((x - m.info.origin.position.x) / res)))

    def cell_to_world(m, cell):
        res = m.info.resolution
        return (m.info.origin.position.x + (cell[1] + 0.5) * res,
                m.info.origin.position.y + (cell[0] + 0.5) * res)

    def stamp_sec(m):
        return m.header.stamp.sec + m.header.stamp.nanosec * 1e-9

    def wait_fresh_map(after_sim, timeout_sim=20.0):
        """Chờ SLAM publish một /map MỚI (stamp > after_sim + 0.5 s) – máy chậm có thể vài giây mô
        phỏng mới có một bản đồ mới; kết luận dựa trên bản đồ cũ sẽ sai."""
        start = sim_now()
        wall_end = time.monotonic() + max(60.0, timeout_sim * 20.0)
        while rclpy.ok() and sim_now() - start < timeout_sim and time.monotonic() < wall_end:
            m = state['map']
            if m is not None and stamp_sec(m) > after_sim + 0.5:
                return True
            spin_for(0.2)
        return False

    def remaining_frontiers(m):
        """[(x, y, chiều_dài_m)] các đoạn biên còn lại mà robot tới gần được, dài nhất trước."""
        pose = robot_pose()
        if m is None or pose is None:
            return []
        try:
            cands, _, _ = find_candidates(grid_of(m), world_to_cell(m, pose[0], pose[1]), m.info.resolution,
                                          P['robot_radius'], min_cells, view_radius=P['view_radius'],
                                          clearance=P['goal_clearance'])
        except Exception:  # noqa: BLE001 – chỉ để báo cáo, không được làm hỏng bước lưu bản đồ
            return []
        out = []
        for c in sorted(cands, key=lambda c: -c.size):
            x, y = cell_to_world(m, (int(round(c.centroid[0])), int(round(c.centroid[1]))))
            out.append((x, y, c.size * m.info.resolution))
        return out

    def explored_area(m):
        g = grid_of(m)
        return float((g >= 0).sum()) * m.info.resolution ** 2

    pkg_share = get_package_share_directory('tb4_lab04')
    maps_dir = P['map_dir'] or resolve_maps_dir(pkg_share, os.path.join(os.getcwd(), 'maps'))
    save_script = os.path.join(get_package_prefix('tb4_lab04'), 'lib', 'tb4_lab04', 'save_map.sh')

    def save(out_dir, label):
        if not P['save_map']:
            return False
        log.info('[{}] Lưu bản đồ vào {} ...'.format(label, out_dir))
        try:
            r = subprocess.run(['bash', save_script, out_dir], timeout=120,
                               stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        except (OSError, subprocess.TimeoutExpired) as exc:
            log.error('Lưu bản đồ thất bại: {}'.format(exc))
            return False
        tail = '\n'.join(r.stdout.strip().splitlines()[-12:])
        if r.returncode == 0:
            log.info('Đã lưu bản đồ:\n' + tail)
            return True
        log.error('save_map.sh lỗi (mã {}):\n{}'.format(r.returncode, tail))
        return False

    def stop_robot():
        for _ in range(3):
            cmd_pub.publish(Twist())
            spin_for(0.05)

    # ---------------- 1. Chờ hệ thống sẵn sàng ----------------
    log.info('Chờ /map (SLAM Toolbox), TF map->base_link và Nav2 (navigate_to_pose) ...')
    deadline = time.monotonic() + P['startup_timeout']
    while rclpy.ok() and time.monotonic() < deadline and (state['map'] is None or robot_pose() is None):
        spin_for(0.2)
    if _ros_shutting_down():   # Ctrl+C lúc đang chờ (rclpy có thể tắt context trước KeyboardInterrupt)
        print('[tb4_lab04] Đã dừng (Ctrl+C).', flush=True)
        return 0
    if state['map'] is None or robot_pose() is None:
        log.error('Không có /map hoặc TF map->base_link sau {:.0f} s – SLAM Toolbox đã chạy chưa?'
                  .format(P['startup_timeout']))
        return 1
    if not nav.wait_for_server(timeout_sec=max(1.0, deadline - time.monotonic())):
        if _ros_shutting_down():
            return 0
        log.error('Không thấy action server navigate_to_pose – Nav2 chưa chạy?')
        return 1

    # ---------------- 2a. Kiểm tra LiDAR có số đo hợp lý ----------------
    if P['check_sensors']:
        end = time.monotonic() + 60.0
        while rclpy.ok() and state['scan'] is None and time.monotonic() < end:
            spin_for(0.2)
        sc = state['scan']
        h = scan_health(sc.ranges, sc.range_min, sc.range_max) if sc is not None else None
        if h is None or h['verdict'] != 'ok':
            log.error('LiDAR (/scan) cho số đo KHÔNG hợp lý: {} -> không khám phá được (SLAM chỉ vẽ '
                      'được vài ô quanh robot). Dừng, không lái, không ghi đè bản đồ.'.format(
                          'không có /scan' if h is None else SCAN_ADVICE[h['verdict']]))
            print(RENDER_FIX, flush=True)
            print('Kiểm tra chi tiết: ros2 run tb4_lab04 check_sensors.py', flush=True)
            return 2
        log.info('LiDAR OK: {} tia, trung vị {:.2f} m, xa nhất {:.2f} m.'.format(
            h['valid'], h['median'], h['max']))

    # ---------------- 2b. Ra khỏi dock (và KIỂM TRA đã ra thật) ----------------
    def odom_pose():
        return lookup_2d('odom', 'base_link')

    def dock_xy():
        d = lookup_2d('odom', 'std_dock_link')
        return dock_body_xy(d if d is not None else DEFAULT_DOCK_POSE)

    def lidar_pose():
        sc = state['scan']
        return lookup_2d('base_link', sc.header.frame_id) if sc is not None else None

    def front_free():
        sc, lp = state['scan'], lidar_pose()
        if sc is None or lp is None:
            return 0.0
        return front_free_distance(scan_to_base(sc, lp), half_width=0.2)

    def drive_until(lin, ang, done, sim_limit):
        """Gửi /cmd_vel 10 Hz tới khi done() đúng hoặc hết sim_limit giây mô phỏng.
        Trả về (done_ok, đã_nhúc_nhích)."""
        tw = Twist()
        tw.linear.x, tw.angular.z = lin, ang
        p0 = odom_pose()
        start = sim_now()
        wall_end = time.monotonic() + max(60.0, sim_limit * 20.0)
        ok = False
        while rclpy.ok() and sim_now() - start < sim_limit and time.monotonic() < wall_end:
            if done():
                ok = True
                break
            cmd_pub.publish(tw)
            spin_for(0.1)
        stop_robot()
        p1 = odom_pose()
        moved = p0 is not None and p1 is not None and (
            math.hypot(p1[0] - p0[0], p1[1] - p0[1]) > 0.02 or abs(wrap_angle(p1[2] - p0[2])) > 0.1)
        return ok, moved

    def escape_dock():
        """Quay mặt khỏi dock rồi tiến ra tới escape_distance (dừng sớm nếu phía trước < 0.3 m)."""
        dxy = dock_xy()
        for attempt in (1, 2):
            p = odom_pose()
            if p is None:
                return False
            target = away_heading(p[:2], dxy)
            err = wrap_angle(target - p[2])
            log.info('Tự thoát khỏi dock (lần {}): quay {:.0f}° cho mặt hướng ra xa dock, rồi tiến '
                     'ra ...'.format(attempt, math.degrees(err)))
            def facing_away():
                q = odom_pose()
                return q is not None and abs(wrap_angle(away_heading(q[:2], dxy) - q[2])) < 0.15

            def far_enough():
                q = odom_pose()
                return q is None or math.hypot(q[0] - dxy[0], q[1] - dxy[1]) >= P['escape_distance']

            _, moved_t = drive_until(0.0, 0.5 if err > 0 else -0.5, facing_away, 40.0)
            _, moved_f = drive_until(0.12, 0.0, lambda: far_enough() or front_free() < 0.30, 30.0)
            q = odom_pose()
            if q is None:
                return False
            ok, d, e = undock_state(q, dxy, P['escape_distance'] - 0.1)
            log.info('Sau khi tự thoát: cách thân dock {:.2f} m, lệch hướng {:.0f}°.'.format(
                d, math.degrees(e)))
            if ok:
                return True
            if not (moved_t or moved_f):
                log.warn('Robot không nhúc nhích -> Create 3 đang chạy phản xạ và bỏ qua /cmd_vel '
                         '(hazard: {}). Chờ {:.0f} s mô phỏng rồi thử lại ...'.format(
                             ', '.join(sorted(state['hz'])) or 'không rõ', P['undock_settle']))
                sim_sleep(P['undock_settle'])
        return False

    def escape_stuck():
        """Robot kẹt (Nav2 thất bại / không tiến): thường do đang tì sát tường – bumper bị ép nên Create 3
        chạy phản xạ và bỏ qua /cmd_vel, còn DWB không bao giờ lùi. Tự gỡ: lùi ra (nếu phía sau trống),
        quay mặt về hướng thoáng nhất theo LiDAR, rồi tiến ra một đoạn ngắn."""
        stop_robot()
        p0 = odom_pose()
        if p0 is None:
            return False
        log.warn('Tự gỡ kẹt: lùi ra, quay về hướng thoáng nhất rồi tiến ra ...')

        def moved(dist):
            def done():
                q = odom_pose()
                return q is None or math.hypot(q[0] - p0[0], q[1] - p0[1]) >= dist
            return done
        # Hành lang kiểm tra trước/sau HẸP hơn thân robot (0.15 < bán kính 0.17 m): khi robot tì SÁT
        # tường bên hông, các điểm của bức tường đó (|y| ~ 0.17-0.2, x ~ 0) không bị tính là "chắn
        # phía trước/sau" – trước đây làm cả 2 hướng = 0 m nên robot không nhúc nhích, dừng vì kẹt.
        def free(fn):
            sc, lp = state['scan'], lidar_pose()
            return 0.0 if sc is None or lp is None else fn(scan_to_base(sc, lp), half_width=0.15)
        diag = {'rear': free(rear_free_distance), 'front': free(front_free_distance), 'open': None}
        if diag['rear'] > 0.35:
            drive_until(-0.08, 0.0, lambda: moved(0.25)() or free(rear_free_distance) < 0.25, 8.0)
        sc, lp = state['scan'], lidar_pose()
        if sc is not None and lp is not None:
            pts = scan_to_base(sc, lp)
            near = nearest_obstacle(pts)
            # Đang tì sát vật cản -> chọn hướng thoáng nhất nhưng QUAY LƯNG lại phía vật cản đó.
            avoid = (near[0], math.radians(100.0)) if near is not None and near[1] < 0.35 else None
            heading, clearance = best_open_heading(pts, avoid=avoid)
            diag['open'] = clearance
            q = odom_pose()
            if q is not None and abs(heading) > 0.15:
                target = wrap_angle(q[2] + heading)

                def facing():
                    r = odom_pose()
                    return r is None or abs(wrap_angle(target - r[2])) < 0.12
                drive_until(0.0, 0.5 if heading > 0 else -0.5, facing, 20.0)
            if free(front_free_distance) > 0.3:
                p1 = odom_pose()

                def forward_done():
                    r = odom_pose()
                    return (r is None or p1 is None or free(front_free_distance) < 0.25 or
                            math.hypot(r[0] - p1[0], r[1] - p1[1]) >= 0.3)
                drive_until(0.12, 0.0, forward_done, 10.0)
        q = odom_pose()
        d = math.hypot(q[0] - p0[0], q[1] - p0[1]) if q is not None else 0.0
        log.info('Tự gỡ kẹt xong: đã dịch {:.2f} m (trống phía sau {:.2f} m, phía trước {:.2f} m, hướng '
                 'thoáng nhất {}).'.format(d, diag['rear'], diag['front'],
                                          'không có scan' if diag['open'] is None
                                          else '{:.2f} m'.format(diag['open'])))
        return d > 0.05

    if P['undock']:
        end = time.monotonic() + 60.0
        while rclpy.ok() and state['dock'] is None and time.monotonic() < end:
            spin_for(0.2)
        if state['dock'] is not None and state['dock'].is_docked:
            log.info('Robot đang trong dock -> undock ...')
            state['hz'].clear()
            client = ActionClient(node, Undock, '/undock')
            status = 'không gửi được'
            if client.wait_for_server(timeout_sec=20.0):
                fut = client.send_goal_async(Undock.Goal())
                rclpy.spin_until_future_complete(node, fut, timeout_sec=20.0)
                gh = fut.result()
                if gh is not None and gh.accepted:
                    res = gh.get_result_async()
                    start = sim_now()
                    wall_end = time.monotonic() + 600.0
                    while rclpy.ok() and not res.done() and sim_now() - start < 60.0 \
                            and time.monotonic() < wall_end:
                        spin_for(0.1)
                    if res.done():
                        status = {GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
                                  GoalStatus.STATUS_CANCELED: 'CANCELED',
                                  GoalStatus.STATUS_ABORTED: 'ABORTED'}.get(
                                      res.result().status, str(res.result().status))
                    else:
                        status = 'quá giờ'
            else:
                log.warn('Không thấy action /undock.')
            log.info('Undock: {}{}. Chờ {:.0f} s mô phỏng cho phản xạ an toàn của Create 3 kết '
                     'thúc ...'.format(status, (' – hazard gây phản xạ: ' + ', '.join(
                         sorted(state['hz']))) if state['hz'] else '', P['undock_settle']))
            sim_sleep(P['undock_settle'])
        # Dù undock báo gì: kiểm tra vị trí THỰC so với dock.
        p = odom_pose()
        if p is not None:
            ok, d, e = undock_state(p, dock_xy(), P['escape_distance'] - 0.1)
            if ok:
                log.info('Đã ra khỏi dock: cách thân dock {:.2f} m, mặt quay ra xa dock.'.format(d))
            else:
                log.warn('Robot CHƯA ra khỏi dock đúng cách (cách thân dock {:.2f} m, lệch hướng '
                         '{:.0f}° – dock đang chắn phía trước).'.format(d, math.degrees(e)))
                if not escape_dock():
                    log.error('Không đưa robot ra khỏi dock được. Thường do cảm biến cliff/bumper báo '
                              'sai liên tục (render đồ hoạ hỏng) -> chạy: ros2 run tb4_lab04 '
                              'check_sensors.py. Dừng, không lưu bản đồ.')
                    print(RENDER_FIX, flush=True)
                    return 3

    # ---------------- 3. Vòng lặp khám phá ----------------
    home = robot_pose()
    t_start = sim_now()
    wall_start = time.monotonic()
    last_autosave = t_start
    attempts = {}          # place_key -> số lần thử
    dead = []              # (x, y) ô frontier đã tới sát mà vẫn không quét được (LiDAR không thấy)
    skipped = []           # (x, y) đã bỏ qua
    # Chỗ đứng không dùng nữa (pick() chọn chỗ đứng khác cho cùng cụm frontier thay vì bỏ cả cụm):
    nogain_views = []      # đã tới nơi nhưng LiDAR không quét thêm được gì – giữ qua lượt quét lại
    failed_views = []      # thử 2 lần không thành (Nav2 thất bại...) – xoá ở lượt quét lại
    view_tries = {}        # place_key(đoạn biên, 1 m) -> số chỗ đứng đã thử mà không quét thêm được
    stats = {'succeeded': 0, 'failed': 0, 'cancelled': 0, 'escapes': 0}
    no_frontier_count = 0
    second_pass = False
    tiny_spins = 0
    tiny_map = False
    res_m = state['map'].info.resolution
    min_cells = max(2, int(round(P['min_frontier_size'] / res_m)))

    def pick():
        m = state['map']
        pose = robot_pose()
        if m is None or pose is None:
            return None, 0, m
        grid = grid_of(m)
        blocked = avoid = None
        if dead:
            blocked = stamp_discs(grid.shape, [world_to_cell(m, x, y) for x, y in dead],
                                  max(1, int(0.25 / m.info.resolution)))
        if nogain_views or failed_views:
            avoid = stamp_discs(grid.shape, [world_to_cell(m, x, y) for x, y in nogain_views + failed_views],
                                max(1, int(0.6 / m.info.resolution)))
        cands, n_groups, _ = find_candidates(grid, world_to_cell(m, pose[0], pose[1]),
                                             m.info.resolution, P['robot_radius'], min_cells,
                                             blocked=blocked, view_radius=P['view_radius'],
                                             clearance=P['goal_clearance'], avoid=avoid)
        good = list(cands)
        good.sort(key=lambda c: score(c, m.info.resolution))
        return (good[0] if good else None), len(good), m

    def send_goal(m, cand, pose):
        gx, gy = cell_to_world(m, cand.goal_cell)
        cx, cy = cell_to_world(m, (int(round(cand.centroid[0])), int(round(cand.centroid[1]))))
        yaw = math.atan2(cy - gy, cx - gx) if math.hypot(cx - gx, cy - gy) > 0.1 else pose[2]
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = node.get_clock().now().to_msg()
        goal.pose.pose.position.x, goal.pose.pose.position.y = gx, gy
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)
        fut = nav.send_goal_async(goal)
        rclpy.spin_until_future_complete(node, fut, timeout_sec=15.0)
        gh = fut.result()
        if gh is None or not gh.accepted:
            return None, (gx, gy)
        return gh, (gx, gy)

    def cancel(gh):
        if gh is None:
            return
        fut = gh.cancel_goal_async()
        rclpy.spin_until_future_complete(node, fut, timeout_sec=10.0)
        stop_robot()

    STOPPED = 'người dùng dừng (Ctrl+C)'
    STUCK_REASON = ('robot kẹt, tự gỡ {} lần liên tiếp không nhúc nhích – dừng và lưu bản đồ hiện có. '
                    'Muốn khám phá tiếp: để launch chạy, lái teleop ra chỗ thoáng, rồi ở terminal khác: '
                    'ros2 run tb4_lab04 auto_explore.py'.format(P['max_stuck_escapes']))
    stuck_streak = 0
    finished_reason = 'không còn frontier đi tới được'
    try:
        while rclpy.ok():
            if sim_now() - t_start > P['max_duration']:
                finished_reason = 'hết max_duration ({:.0f} s mô phỏng)'.format(P['max_duration'])
                break
            if P['autosave_period'] > 0 and sim_now() - last_autosave > P['autosave_period']:
                save(os.path.join(maps_dir, 'autosave'), 'tự lưu định kỳ')
                last_autosave = sim_now()

            cand, n_good, m = pick()
            if cand is None and m is not None and explored_area(m) < P['min_explored_area']:
                # Bản đồ mới vài m² mà đã "hết frontier": không thể là khám phá xong. Quay 1 vòng
                # tại chỗ cho LiDAR quét đủ 360 độ; vẫn vậy sau 2 lần -> lỗi cảm biến/SLAM.
                tiny_spins += 1
                if tiny_spins > 2:
                    tiny_map = True
                    finished_reason = ('LỖI: bản đồ chỉ có {:.1f} m² dù đã quay quét 2 vòng – LiDAR/'
                                       'SLAM không hoạt động đúng'.format(explored_area(m)))
                    break
                log.warn('Bản đồ mới {:.1f} m² và chưa có frontier đi tới được -> quay 1 vòng tại '
                         'chỗ để quét ({}/2) ...'.format(explored_area(m), tiny_spins))
                acc = {'yaw': 0.0, 'prev': None}

                def full_turn():
                    q = odom_pose()
                    if q is None:
                        return False
                    if acc['prev'] is not None:
                        acc['yaw'] += abs(wrap_angle(q[2] - acc['prev']))
                    acc['prev'] = q[2]
                    return acc['yaw'] >= 2.0 * math.pi

                drive_until(0.0, 0.6, full_turn, 40.0)
                wait_fresh_map(sim_now())
                continue
            if cand is None:
                no_frontier_count += 1
                log.info('Không còn frontier đi tới được ({}/{}) – chờ SLAM cập nhật bản đồ ...'
                         .format(no_frontier_count, P['done_confirmations']))
                if no_frontier_count >= P['done_confirmations']:
                    if not second_pass and (dead or attempts):
                        # Lượt quét lại: các đoạn biên đã bị đánh dấu "không quét được" / vị trí đã thử
                        # 2 lần thường được bỏ qua lúc robot còn ở chỗ khác hoặc bản đồ còn thô. Thử lại
                        # MỘT lần trên bản đồ hiện tại (đầy đủ hơn) trước khi kết luận – lấp các góc khuyết.
                        second_pass = True
                        log.info('Hết frontier – quét lại 1 lượt các vùng đã bỏ qua ({} đoạn biên, {} vị '
                                 'trí) ...'.format(len(dead), len(attempts)))
                        dead.clear()
                        attempts.clear()
                        failed_views.clear()
                        view_tries.clear()   # nogain_views giữ lại -> lượt này nhìn từ chỗ đứng KHÁC
                        skipped.clear()      # chỉ báo những chỗ vẫn bỏ qua sau lượt quét lại
                        no_frontier_count = 0
                        continue
                    break
                sim_sleep(P['replan_period'])
                continue
            no_frontier_count = 0
            pose = robot_pose()
            gain_r = int(1.5 / m.info.resolution)
            known_before = known_cells_around(grid_of(m), cand.frontier_cell, gain_r)
            gh, (gx, gy) = send_goal(m, cand, pose)
            key = place_key(gx, gy)
            attempts[key] = attempts.get(key, 0) + 1
            if gh is None:
                log.warn('Nav2 từ chối goal ({:.2f}, {:.2f}) – thử lại sau.'.format(gx, gy))
                stats['failed'] += 1
                stats['escapes'] += 1
                stuck_streak = 0 if escape_stuck() else stuck_streak + 1
                if stuck_streak >= P['max_stuck_escapes']:
                    finished_reason = STUCK_REASON
                    break
                sim_sleep(3.0)
                continue
            log.info('Goal -> ({:.2f}, {:.2f}) | cụm frontier {:.1f} m | đường đi ~{:.1f} m | '
                     'còn {} cụm | đã quét {:.0f} m²'.format(
                         gx, gy, cand.size * res_m, cand.path_cells * res_m, n_good,
                         explored_area(m)))
            res_fut = gh.get_result_async()
            g_start = sim_now()
            best_d = math.hypot(gx - pose[0], gy - pose[1])
            last_progress = g_start
            last_check = g_start
            outcome = None
            while rclpy.ok():
                spin_for(0.2)
                if res_fut.done():
                    status = res_fut.result().status
                    outcome = 'succeeded' if status == GoalStatus.STATUS_SUCCEEDED else 'failed'
                    break
                now = sim_now()
                p = robot_pose()
                if p is not None:
                    d = math.hypot(gx - p[0], gy - p[1])
                    if d < best_d - 0.2:
                        best_d, last_progress = d, now
                if now - g_start > P['goal_timeout']:
                    log.warn('Goal quá {:.0f} s mô phỏng -> huỷ.'.format(P['goal_timeout']))
                    cancel(gh)
                    outcome = 'failed'
                    break
                if now - last_progress > P['no_progress_timeout']:
                    log.warn('Robot không tiến thêm trong {:.0f} s mô phỏng -> huỷ goal.'.format(
                        P['no_progress_timeout']))
                    cancel(gh)
                    outcome = 'failed'
                    break
                if now - last_check > P['replan_period']:
                    last_check = now
                    mm = state['map']
                    fcell = world_to_cell(mm, *cell_to_world(m, cand.frontier_cell))
                    if not frontier_still_there(grid_of(mm), fcell, int(0.6 / mm.info.resolution)):
                        log.info('Cụm frontier đã được quét xong trước khi tới -> chọn cụm khác.')
                        cancel(gh)
                        attempts[key] -= 1
                        outcome = 'cancelled'
                        break
            if outcome is None:          # rclpy bị tắt giữa chừng (Ctrl+C)
                break
            stats[outcome] += 1
            if outcome == 'failed' and rclpy.ok():
                stats['escapes'] += 1
                stuck_streak = 0 if escape_stuck() else stuck_streak + 1
                if stuck_streak >= P['max_stuck_escapes']:
                    # Kẹt thật (tự gỡ nhiều lần không nhúc nhích): dừng và LƯU bản đồ đang có, thay vì
                    # gửi goal thất bại tới hết max_duration.
                    finished_reason = STUCK_REASON
                    break
            elif outcome == 'succeeded':
                stuck_streak = 0
            if outcome == 'succeeded':
                # Đã tới sát mà frontier vẫn còn VÀ quanh đó gần như không có thêm ô nào được quét
                # -> LiDAR không nhìn vào được (khe tường, bị thân robot che) -> đánh dấu "chết".
                # Nếu có quét thêm (dù biên vẫn còn) thì KHÔNG đánh dấu – vẫn là vùng khám phá được.
                # Phải xét trên bản đồ MỚI.
                fresh = wait_fresh_map(sim_now())
                mm = state['map']
                fcell = world_to_cell(mm, *cell_to_world(m, cand.frontier_cell))
                gm = grid_of(mm)
                gained = (known_cells_around(gm, fcell, gain_r) - known_before) \
                    * mm.info.resolution ** 2
                if fresh and gained < 0.1 and frontier_still_there(
                        gm, fcell, int(0.3 / mm.info.resolution)):
                    # Chỗ đứng này không nhìn vào được (góc tường che, khe hẹp...) -> lần sau chọn chỗ
                    # đứng KHÁC cho cùng đoạn biên (thường là vòng sang phía khác). Chỉ khi đã thử
                    # max_views_per_frontier chỗ mà vẫn không được mới bỏ đoạn biên (bán kính 1 m).
                    nogain_views.append((gx, gy))
                    fkey = place_key(*cell_to_world(m, cand.frontier_cell), cell=1.0)
                    view_tries[fkey] = view_tries.get(fkey, 0) + 1
                    if view_tries[fkey] >= P['max_views_per_frontier']:
                        fr, fc = cand.frontier_cell
                        lim = (1.0 / m.info.resolution) ** 2
                        for r, q in cand.cells:
                            if (int(r) - fr) ** 2 + (int(q) - fc) ** 2 <= lim:
                                dead.append(cell_to_world(m, (int(r), int(q))))
                        log.info('Đã thử {} chỗ đứng mà LiDAR vẫn không quét thêm được đoạn biên này '
                                 '-> bỏ qua đoạn đó.'.format(view_tries[fkey]))
                    else:
                        log.info('Chỗ đứng này không nhìn vào được đoạn biên -> lần sau thử chỗ đứng '
                                 'khác ({}/{}).'.format(view_tries[fkey], P['max_views_per_frontier']))
            if outcome != 'cancelled' and attempts[key] >= 2:
                failed_views.append((gx, gy))
                skipped.append((gx, gy))
                log.warn('Bỏ chỗ đứng ({:.2f}, {:.2f}) sau 2 lần thử không được -> sẽ nhìn đoạn biên từ chỗ '
                         'khác.'.format(gx, gy))
            sim_sleep(1.0)
    except (KeyboardInterrupt, ExternalShutdownException):
        # Ctrl+C: `ros2 run ... auto_explore.py` -> vẫn thử lưu bên dưới (SLAM ở terminal khác còn sống);
        # explore.launch.py (save_on_ctrl_c=False) -> thoát ngay, bản đồ dự phòng ở maps/autosave/.
        # (finished_reason được khởi tạo sẵn = "không còn frontier" -> phải ghi đè, nếu không Ctrl+C bị
        # coi như khám phá xong: báo sai lý do và còn cho robot quay về điểm xuất phát.)
        finished_reason = STOPPED
    if not rclpy.ok() or finished_reason is None:
        finished_reason = STOPPED
    if finished_reason == STOPPED and not P['save_on_ctrl_c']:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        auto = os.path.join(maps_dir, 'autosave', 'map.yaml')
        print('[tb4_lab04] Đã dừng (Ctrl+C) – SLAM Toolbox tắt cùng launch nên KHÔNG lưu bản đồ lúc này.\n'
              '  Bản tự lưu gần nhất (mỗi {:.0f} s mô phỏng): {}\n'
              '  Muốn giữ bản đồ hiện tại: lần sau chạy `ros2 run tb4_lab04 save_map.sh` ở terminal khác '
              'TRƯỚC khi Ctrl+C.'.format(P['autosave_period'],
                                        auto if os.path.exists(auto) else '(chưa có)'), flush=True)
        try:
            node.destroy_node()
        except Exception:  # noqa: BLE001
            pass
        return 0

    # ---------------- 4. Về điểm xuất phát + lưu ----------------
    try:
        if P['return_home'] and home is not None and rclpy.ok() \
                and finished_reason not in (STOPPED, STUCK_REASON) \
                and not tiny_map:
            log.info('Quay về điểm xuất phát ({:.2f}, {:.2f}) để SLAM khép vòng ...'.format(
                home[0], home[1]))
            goal = NavigateToPose.Goal()
            goal.pose.header.frame_id = 'map'
            goal.pose.header.stamp = node.get_clock().now().to_msg()
            goal.pose.pose.position.x, goal.pose.pose.position.y = home[0], home[1]
            goal.pose.pose.orientation.z = math.sin(home[2] / 2.0)
            goal.pose.pose.orientation.w = math.cos(home[2] / 2.0)
            fut = nav.send_goal_async(goal)
            rclpy.spin_until_future_complete(node, fut, timeout_sec=15.0)
            gh = fut.result()
            if gh is not None and gh.accepted:
                res = gh.get_result_async()
                start = sim_now()
                while rclpy.ok() and not res.done() and sim_now() - start < P['goal_timeout']:
                    spin_for(0.2)
                if not res.done():
                    cancel(gh)
            sim_sleep(3.0)   # để SLAM xử lý vài scan cuối
        if rclpy.ok():
            stop_robot()
        else:
            print('[tb4_lab04] Đã dừng – thử lưu bản đồ hiện có (cần SLAM Toolbox còn chạy) ...')
        m = state['map']
        free_area = (float((grid_of(m) >= 0).sum() - (grid_of(m) >= OCC_THRESH).sum())
                     * m.info.resolution ** 2) if m is not None else 0.0
        if free_area < P['min_save_area']:
            log.error('Vùng trống trên bản đồ chỉ {:.1f} m² (< {:.0f} m²) -> KHÔNG lưu để khỏi ghi đè '
                      'bản đồ cũ bằng bản đồ hỏng.'.format(free_area, P['min_save_area']))
            if tiny_map:
                print(RENDER_FIX, flush=True)
            saved = False
        else:
            saved = save(maps_dir, 'lưu bản đồ cuối')
        dur = sim_now() - t_start
        print('\n================ KẾT QUẢ TỰ KHÁM PHÁ ================')
        print('  Kết thúc vì       : {}'.format(finished_reason))
        print('  Thời gian         : {:.0f} s mô phỏng ({:.0f} s thực)'.format(
            dur, time.monotonic() - wall_start))
        print('  Diện tích đã quét : {:.0f} m²'.format(explored_area(m) if m else 0.0))
        print('  Goal              : {succeeded} tới nơi, {failed} thất bại, {cancelled} huỷ '
              '(cụm đã quét xong), {escapes} lần tự gỡ kẹt'.format(**stats))
        left = remaining_frontiers(m)
        if left:
            print('  Biên chưa quét    : ' + ', '.join('({:.1f}, {:.1f}) dài {:.1f} m'.format(*f)
                                                     for f in left[:8]))
            print('    -> toạ độ frame map (RViz: Publish Point để xem). Muốn lấp: để launch chạy, lái teleop tới '
                  'đó rồi `ros2 run tb4_lab04 save_map.sh` (hoặc chạy lại auto_explore.py).')
        print('  Bản đồ            : {}'.format(
            os.path.join(maps_dir, 'map.yaml') if saved else 'CHƯA LƯU ĐƯỢC – chạy save_map.sh'))
        print('=====================================================')
        if saved:
            print('Xong. Có thể Ctrl+C để tắt mô phỏng, rồi làm Phần 3.')
        if finished_reason == STOPPED:
            return 0   # người dùng chủ động dừng: không để launch báo "[ERROR] process has died"
        return 0 if saved or not P['save_map'] else 1
    except (KeyboardInterrupt, ExternalShutdownException):
        return 0 if _SIGINT['got'] else 1
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)   # SIGINT lần 2 (launch) -> bỏ qua
        try:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:  # noqa: BLE001 – đang thoát, không cần báo thêm
            pass


_SIGINT = {'got': False}


def _on_sigint(signum, frame):
    """Như mặc định (KeyboardInterrupt) nhưng ghi nhớ là người dùng đã bấm Ctrl+C."""
    _SIGINT['got'] = True
    raise KeyboardInterrupt


signal.signal(signal.SIGINT, _on_sigint)
# launch leo thang SIGINT -> SIGTERM -> SIGKILL. rclpy chỉ tắt context khi nhận SIGTERM (không thoát) nên
# nếu đang kẹt trong bước lưu bản đồ thì bị SIGKILL sau 15 s -> xử lý SIGTERM giống Ctrl+C.
signal.signal(signal.SIGTERM, _on_sigint)


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
        return 0
    except Exception as exc:  # noqa: BLE001
        if type(exc).__name__ == 'ExternalShutdownException' or _ros_shutting_down():   # đang tắt (Ctrl+C)
            signal.signal(signal.SIGINT, signal.SIG_IGN)
            return 0
        raise


if __name__ == '__main__':
    sys.exit(_run())
