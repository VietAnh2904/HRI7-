#!/usr/bin/env python3
"""Làm sạch bản đồ đã lưu: xoá "vệt trắng" bên trong khối tường và bịt khe hở 1-2 ô trên tường.

    ros2 run tb4_lab04 clean_map.py                    # maps/map.yaml (giống save_map.sh)
    ros2 run tb4_lab04 clean_map.py /duong/dan/map.yaml
    ros2 run tb4_lab04 clean_map.py --dry-run          # chỉ báo sẽ sửa bao nhiêu, không ghi

Vì sao có vệt trắng? Tường mê cung trong Gazebo là các khối hộp ghép lại. Ở chỗ ghép/góc tường có
khe rất nhỏ; tia LiDAR lọt qua khe sẽ đi tiếp vào BÊN TRONG khối tường cho tới mặt đối diện. SLAM
Toolbox đánh dấu mọi ô tia đi qua là "trống" -> trên map.pgm thấy vệt/quạt trắng trong vùng xám.

Script làm 2 việc (không đụng tới vùng robot đi được):
  1. Bịt khe trên tường rộng <= 2 ô (0.1 m) bằng phép đóng hình thái (dilate -> erode) trên ô vật cản:
     ô trống nằm trong khe -> thành vật cản.
  2. Ô trống KHÔNG liền (4 hướng) với vùng trống chính (vùng chứa điểm xuất phát (0, 0), nếu không có
     thì vùng lớn nhất) -> thành "chưa biết" (xám), giống phần còn lại bên trong khối tường.

Bản gốc được chép vào maps/backup_<thời_gian>_truoc_clean/ trước khi ghi đè. Đây là bước HẬU XỬ LÝ
bản đồ – nếu dùng cho bài nộp, hãy ghi rõ trong báo cáo.
"""

import argparse
import datetime
import os
import shutil
import sys

import numpy as np
import yaml

FREE, UNKNOWN, OCC = 254, 205, 0


def shift(mask, dy, dx):
    out = np.zeros_like(mask)
    h, w = mask.shape
    ys, yd = (slice(0, h - dy), slice(dy, h)) if dy >= 0 else (slice(-dy, h), slice(0, h + dy))
    xs, xd = (slice(0, w - dx), slice(dx, w)) if dx >= 0 else (slice(-dx, w), slice(0, w + dx))
    out[yd, xd] = mask[ys, xs]
    return out


def dilate8(mask):
    out = mask.copy()
    for dy in (-1, 0, 1):
        for dx in (-1, 0, 1):
            if dy or dx:
                out |= shift(mask, dy, dx)
    return out


def erode8(mask):
    return ~dilate8(~mask)


def flood4(passable, seed):
    """Vùng liền 4 hướng chứa seed trong mảng bool passable."""
    reached = np.zeros(passable.shape, dtype=bool)
    if seed is None or not passable[seed]:
        return reached
    reached[seed] = True
    front = reached.copy()
    while front.any():
        grown = shift(front, 1, 0) | shift(front, -1, 0) | shift(front, 0, 1) | shift(front, 0, -1)
        front = grown & passable & ~reached
        reached |= front
    return reached


def largest_component_seed(passable):
    """Ô bất kỳ thuộc vùng liền lớn nhất (dùng khi điểm xuất phát không nằm trên ô trống)."""
    remaining = passable.copy()
    best, best_n = None, 0
    while remaining.any():
        seed = tuple(int(v) for v in np.argwhere(remaining)[0])
        comp = flood4(remaining, seed)
        n = int(comp.sum())
        if n > best_n:
            best, best_n = seed, n
        remaining &= ~comp
    return best


def clean(img, seed_rc):
    """img: mảng uint8 trinary (254 trống / 205 chưa biết / 0 vật cản), hàng 0 = phía trên ảnh.

    Trả về (ảnh_mới, số_ô_khe_đã_bịt, số_ô_trống_bị_xoá).
    """
    occ = img == OCC
    free = img == FREE
    closed = erode8(dilate8(occ))           # đóng hình thái: bịt khe <= 2 ô
    plugged = closed & free                 # ô trống nằm trong khe tường
    passable = free & ~closed
    h, w = img.shape
    seed = seed_rc if (seed_rc is not None and 0 <= seed_rc[0] < h and 0 <= seed_rc[1] < w
                       and passable[seed_rc]) else None
    if seed is None:
        # điểm xuất phát có thể nằm sát tường -> tìm ô đi được gần nhất trong bán kính 10 ô
        if seed_rc is not None:
            pts = np.argwhere(passable)
            if len(pts):
                d2 = (pts[:, 0] - seed_rc[0]) ** 2 + (pts[:, 1] - seed_rc[1]) ** 2
                i = int(np.argmin(d2))
                if d2[i] <= 100:
                    seed = (int(pts[i, 0]), int(pts[i, 1]))
        if seed is None:
            seed = largest_component_seed(passable)
    main = flood4(passable, seed)
    orphan = passable & ~main
    out = img.copy()
    out[plugged] = OCC
    out[orphan] = UNKNOWN
    return out, int(plugged.sum()), int(orphan.sum())


def read_pgm_array(path):
    here = os.path.dirname(os.path.abspath(__file__))
    sys.path.insert(0, here)
    from check_map import read_pgm   # cùng thư mục scripts/ (hoặc lib/tb4_lab04 sau khi build)
    w, h, maxval, px = read_pgm(path)
    if maxval != 255:
        raise ValueError('Chỉ hỗ trợ PGM 8 bit (maxval 255) do map_saver tạo')
    return np.asarray(px, dtype=np.uint8).reshape(h, w)


def write_pgm(path, img):
    h, w = img.shape
    with open(path, 'wb') as f:
        f.write('P5\n# CREATOR: tb4_lab04 clean_map.py\n{} {}\n255\n'.format(w, h).encode())
        f.write(img.astype(np.uint8).tobytes())


def default_map():
    try:
        from ament_index_python.packages import get_package_share_directory
        share = get_package_share_directory('tb4_lab04')
        with open(os.path.join(share, 'source_dir.txt'), encoding='utf-8') as f:
            return os.path.join(f.read().strip(), 'maps', 'map.yaml')
    except Exception:  # noqa: BLE001
        return os.path.join('maps', 'map.yaml')


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('map', nargs='?', default=None, help='đường dẫn map.yaml')
    ap.add_argument('--dry-run', action='store_true', help='chỉ báo cáo, không ghi file')
    ap.add_argument('--start', nargs=2, type=float, default=[0.0, 0.0], metavar=('X', 'Y'),
                    help='điểm (m) nằm trong vùng trống chính – mặc định (0, 0) = chỗ robot xuất phát')
    a = ap.parse_args(argv)
    yaml_path = os.path.abspath(a.map or default_map())
    with open(yaml_path, 'r', encoding='utf-8') as f:
        meta = yaml.safe_load(f)
    if str(meta.get('mode', 'trinary')) != 'trinary' or int(meta.get('negate', 0)) != 0:
        print('[LỖI] Chỉ hỗ trợ map mode trinary, negate 0 (đúng như save_map.sh tạo).')
        return 1
    image = str(meta['image'])
    if not os.path.isabs(image):
        image = os.path.join(os.path.dirname(yaml_path), image)
    img = read_pgm_array(image)
    vals = set(np.unique(img).tolist())
    if not vals <= {FREE, UNKNOWN, OCC}:
        print('[LỖI] Ảnh có giá trị ngoài 0/205/254 – không phải map trinary của map_saver.')
        return 1
    res = float(meta['resolution'])
    ox, oy = float(meta['origin'][0]), float(meta['origin'][1])
    h = img.shape[0]
    seed = (h - 1 - int((a.start[1] - oy) / res), int((a.start[0] - ox) / res))  # hàng 0 = trên
    out, n_plug, n_orphan = clean(img, seed)
    print('Bản đồ      : {}'.format(yaml_path))
    print('Khe tường bịt lại           : {} ô ({:.2f} m²)'.format(n_plug, n_plug * res * res))
    print('Vệt trắng trong tường đã xoá: {} ô ({:.2f} m²)'.format(n_orphan, n_orphan * res * res))
    if a.dry_run:
        print('(--dry-run: không ghi file)')
        return 0
    if n_plug == 0 and n_orphan == 0:
        print('Không có gì cần sửa.')
        return 0
    bk = os.path.join(os.path.dirname(yaml_path), 'backup_{}_truoc_clean'.format(
        datetime.datetime.now().strftime('%Y%m%d_%H%M%S')))
    os.makedirs(bk, exist_ok=True)
    shutil.copy2(yaml_path, bk)
    shutil.copy2(image, bk)
    tmp = image + '.tmp'
    write_pgm(tmp, out)
    os.replace(tmp, image)
    print('Đã ghi đè {} (bản gốc: {})'.format(image, bk))
    return 0


if __name__ == '__main__':
    sys.exit(main())
