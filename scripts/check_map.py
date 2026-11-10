#!/usr/bin/env python3
# Bài thực hành 04 – kiểm tra nhanh file bản đồ đã lưu (map.yaml + map.pgm).
#
#   ros2 run tb4_lab04 check_map.py                       # tự tìm maps/map.yaml của repo
#   ros2 run tb4_lab04 check_map.py /duong/dan/map.yaml
#   python3 scripts/check_map.py maps/map.yaml            # chạy được cả khi không có ROS
#
# In ra: kích thước, độ phân giải, tỉ lệ ô trống/vật cản/chưa biết, diện tích đã khám phá.
# Không cần ROS, không cần thư viện ngoài ngoài PyYAML.

import os
import sys

import yaml


def read_pgm(path):
    """Đọc ảnh PGM (P5 nhị phân hoặc P2 ASCII). Trả về (width, height, maxval, list pixel)."""
    with open(path, 'rb') as f:
        data = f.read()
    tokens, pos = [], 0
    # Header: magic, width, height, maxval – có thể xen comment '#'
    while len(tokens) < 4:
        while pos < len(data) and data[pos:pos + 1].isspace():
            pos += 1
        if pos >= len(data):
            raise ValueError('PGM header bị cắt cụt')
        if data[pos:pos + 1] == b'#':
            while pos < len(data) and data[pos:pos + 1] not in (b'\n', b'\r'):
                pos += 1
            continue
        start = pos
        while pos < len(data) and not data[pos:pos + 1].isspace():
            pos += 1
        tokens.append(data[start:pos])
    magic = tokens[0]
    width, height, maxval = int(tokens[1]), int(tokens[2]), int(tokens[3])
    if width <= 0 or height <= 0 or not 0 < maxval < 65536:
        raise ValueError('PGM header không hợp lệ')
    n = width * height
    if magic == b'P5':
        pos += 1  # đúng 1 ký tự trắng sau maxval
        bpp = 1 if maxval < 256 else 2
        raw = data[pos:pos + n * bpp]
        if len(raw) < n * bpp:
            raise ValueError('PGM thiếu dữ liệu ({} / {} byte)'.format(len(raw), n * bpp))
        if bpp == 1:
            pixels = list(raw)
        else:
            pixels = [(raw[i] << 8) | raw[i + 1] for i in range(0, n * 2, 2)]
    elif magic == b'P2':
        vals = data[pos:].split()
        if len(vals) < n:
            raise ValueError('PGM thiếu dữ liệu')
        pixels = [int(v) for v in vals[:n]]
    else:
        raise ValueError('Không phải PGM (magic = {!r})'.format(magic))
    return width, height, maxval, pixels


def classify(pixels, maxval, negate, occupied_thresh, free_thresh):
    """Phân loại giống nav2_map_server (mode trinary/scale): trả về (free, occupied, unknown)."""
    free = occ = unk = 0
    for v in pixels:
        shade = v / float(maxval)
        p = shade if negate else 1.0 - shade   # xác suất bị chiếm
        if p > occupied_thresh:
            occ += 1
        elif p < free_thresh:
            free += 1
        else:
            unk += 1
    return free, occ, unk


def unknown_loads_as_free(free_thresh, negate=0):
    """map_saver ghi ô chưa biết = xám 205. True nếu nav2_map_server sẽ đọc nó thành ô trống."""
    shade = 205 / 255.0
    p = shade if negate else 1.0 - shade   # ~0.196
    return p < free_thresh


def analyze(yaml_path):
    with open(yaml_path, 'r', encoding='utf-8') as f:
        meta = yaml.safe_load(f)
    if not isinstance(meta, dict):
        raise ValueError('map.yaml không đúng định dạng')
    image = str(meta['image'])
    if not os.path.isabs(image):
        image = os.path.join(os.path.dirname(os.path.abspath(yaml_path)), image)
    w, h, maxval, px = read_pgm(image)
    res = float(meta['resolution'])
    negate = int(meta.get('negate', 0))
    free_thresh = float(meta.get('free_thresh', 0.25))
    free, occ, unk = classify(px, maxval, negate, float(meta.get('occupied_thresh', 0.65)),
                              free_thresh)
    return {
        'yaml': os.path.abspath(yaml_path), 'image': image, 'width': w, 'height': h,
        'resolution': res, 'origin': meta.get('origin'), 'mode': meta.get('mode', 'trinary'),
        'free_thresh': free_thresh, 'negate': negate,
        'free': free, 'occupied': occ, 'unknown': unk, 'cells': w * h,
        'known_area_m2': (free + occ) * res * res, 'free_area_m2': free * res * res,
    }


def default_map_path():
    """maps/map.yaml trong thư mục source (đọc từ share/tb4_lab04/source_dir.txt) nếu có."""
    try:
        from ament_index_python.packages import get_package_share_directory
        share = get_package_share_directory('tb4_lab04')
        with open(os.path.join(share, 'source_dir.txt'), encoding='utf-8') as f:
            return os.path.join(f.read().strip(), 'maps', 'map.yaml')
    except Exception:  # noqa: BLE001 – không có ROS/không build: dùng thư mục hiện tại
        return os.path.join('maps', 'map.yaml')


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    argv = [a for a in argv if not a.startswith('__')]  # bỏ tham số ROS lạ nếu có
    path = argv[0] if argv else default_map_path()
    if not os.path.isfile(path):
        print('LỖI: không tìm thấy {}'.format(path))
        return 1
    try:
        r = analyze(path)
    except (OSError, KeyError, ValueError, yaml.YAMLError) as exc:
        print('LỖI: bản đồ hỏng hoặc không đọc được: {}'.format(exc))
        return 1
    pct = (lambda k: 100.0 * r[k] / r['cells'])
    print('Bản đồ     : {}'.format(r['yaml']))
    print('Ảnh        : {} ({} x {} ô, mode={})'.format(
        r['image'], r['width'], r['height'], r['mode']))
    print('Độ phân giải: {:.3f} m/ô  ->  {:.1f} m x {:.1f} m'.format(
        r['resolution'], r['width'] * r['resolution'], r['height'] * r['resolution']))
    print('Origin     : {}'.format(r['origin']))
    print('Ô trống    : {:6.2f} %   ({:.1f} m²)'.format(pct('free'), r['free_area_m2']))
    print('Ô vật cản  : {:6.2f} %'.format(pct('occupied')))
    print('Chưa biết  : {:6.2f} %'.format(pct('unknown')))
    print('Diện tích đã khám phá (trống + vật cản): {:.1f} m²'.format(r['known_area_m2']))
    ok = r['free'] > 0 and r['occupied'] > 0
    if unknown_loads_as_free(r['free_thresh'], r['negate']):
        print('CẢNH BÁO: free_thresh = {} quá cao – ô "chưa biết" (xám 205) sẽ bị nav2_map_server '
              'đọc thành ô TRỐNG. Lưu lại bằng `ros2 run tb4_lab04 save_map.sh` (dùng --free 0.15) '
              'hoặc sửa free_thresh trong map.yaml thành 0.15.'.format(r['free_thresh']))
        ok = False
    if r['free'] == 0 or r['occupied'] == 0:
        print('CẢNH BÁO: bản đồ không có ô trống hoặc không có vật cản – có thể lưu khi SLAM '
              'chưa chạy/chưa nhận scan.')
    elif r['free_area_m2'] < 20.0:
        print('CẢNH BÁO: vùng trống < 20 m² – có vẻ robot chưa khảo sát đủ môi trường.')
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
