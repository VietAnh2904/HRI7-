#!/usr/bin/env python3
"""Thêm vật thể (người, xe, ghế, trụ cứu hoả, bình chữa cháy...) vào world Gazebo ĐANG CHẠY.

  ros2 run tb4_lab04 spawn_objects.py --download-only   # 1 lần, cần Internet: tải model về cache
  ros2 run tb4_lab04 spawn_objects.py                   # spawn theo config/objects.yaml
  ros2 run tb4_lab04 spawn_objects.py --list            # chỉ in danh sách + trạng thái cache

navigation.launch.py tự chạy script này ngay khi Gazebo sẵn sàng (tham số spawn_objects:=true, mặc
định). Danh sách vật thể, toạ độ: config/objects.yaml.

Mỗi vật thể:
  1. nếu model CHƯA có trong cache Fuel (~/.ignition/fuel) -> `ign fuel download -u <uri>` (ngoài
     Gazebo: nếu để Gazebo tự tải thì vòng lặp mô phỏng bị chặn tới khi tải xong -> /clock đứng);
  2. `ros2 run ros_gz_sim create -world <world> -file <uri> -name <tên> -x -y -z -Y`.
Không tải được (mất mạng) -> bỏ qua vật thể đó, báo rõ; mô phỏng vẫn chạy bình thường.
Mã thoát: 0 = spawn đủ, 1 = thiếu ít nhất một vật thể, 2 = lỗi cấu hình.
"""

import argparse
import os
import shutil
import subprocess
import sys
import time
from urllib.parse import unquote, urlparse

import yaml

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
import lab_common as lc  # noqa: E402

FUEL_HOST_ALIASES = ('fuel.gazebosim.org', 'fuel.ignitionrobotics.org')


# ------------------------------------------------------------------ logic thuần (có unit test)
def parse_fuel_uri(uri):
    """'https://fuel.gazebosim.org/1.0/OpenRobotics/models/Standing person[/3]'
    -> (host, owner, model_name) hoặc None nếu không phải URI model Fuel."""
    u = urlparse(uri.strip())
    if u.scheme not in ('http', 'https') or not u.netloc:
        return None
    parts = [unquote(p) for p in u.path.split('/') if p]
    # [1.0, owner, models, name, (version)]
    if len(parts) < 4 or parts[2].lower() != 'models':
        return None
    return u.netloc.lower(), parts[1], parts[3]


def fuel_cache_roots():
    roots = []
    env = os.environ.get('IGN_FUEL_CACHE_PATH') or os.environ.get('GZ_FUEL_CACHE_PATH')
    if env:
        roots.append(os.path.expanduser(env))
    roots += [os.path.expanduser('~/.ignition/fuel'), os.path.expanduser('~/.gz/fuel')]
    return roots


def _match_dir(parent, name):
    """Thư mục con của parent trùng tên (không phân biệt hoa thường), hoặc None."""
    try:
        for entry in os.listdir(parent):
            if entry.lower() == name.lower() and os.path.isdir(os.path.join(parent, entry)):
                return os.path.join(parent, entry)
    except OSError:
        pass
    return None


def cached_model_dir(uri, roots=None):
    """Thư mục phiên bản model trong cache Fuel (chứa model.config), hoặc None."""
    parsed = parse_fuel_uri(uri)
    if parsed is None:
        return None
    host, owner, name = parsed
    hosts = [host] + [h for h in FUEL_HOST_ALIASES if h != host]
    for root in roots or fuel_cache_roots():
        for h in hosts:
            d = _match_dir(root, h)
            d = d and _match_dir(d, owner)
            d = d and _match_dir(d, 'models')
            d = d and _match_dir(d, name)
            if not d:
                continue
            versions = []
            for v in os.listdir(d):
                vd = os.path.join(d, v)
                if os.path.isfile(os.path.join(vd, 'model.config')) or \
                        os.path.isfile(os.path.join(vd, 'model.sdf')):
                    versions.append((int(v) if v.isdigit() else -1, vd))
            if versions:
                return max(versions)[1]
    return None


def create_command(world, obj, runner):
    """Dòng lệnh spawn một vật thể bằng `ros2 run <runner> create` (runner: ros_gz_sim/ros_ign_gazebo)."""
    return ['ros2', 'run', runner, 'create',
            '-world', str(world),
            '-file', str(obj['uri']).strip(),
            '-name', str(obj['name']),
            '-x', '{:.3f}'.format(float(obj['x'])),
            '-y', '{:.3f}'.format(float(obj['y'])),
            '-z', '{:.3f}'.format(float(obj.get('z', 0.0))),
            '-Y', '{:.4f}'.format(float(obj.get('yaw', 0.0)))]


def load_objects_file(path):
    with open(path, 'r', encoding='utf-8') as f:
        data = yaml.safe_load(f) or {}
    objs = data.get('objects') or []
    names = set()
    for o in objs:
        for key in ('name', 'uri', 'x', 'y'):
            if key not in o:
                raise ValueError('{}: vật thể thiếu khoá "{}": {}'.format(path, key, o))
        if o['name'] in names:
            raise ValueError('{}: trùng tên vật thể "{}"'.format(path, o['name']))
        names.add(o['name'])
    return str(data.get('world', 'maze')), objs


# ------------------------------------------------------------------ chạy lệnh
def run(cmd, timeout):
    try:
        p = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=timeout,
                           text=True)
        return p.returncode, p.stdout
    except subprocess.TimeoutExpired as exc:
        out = exc.stdout.decode(errors='replace') if isinstance(exc.stdout, bytes) else (exc.stdout or '')
        return None, out
    except OSError as exc:
        return None, str(exc)


def download(uri, timeout):
    if not shutil.which('ign'):
        return False, 'không có lệnh `ign` (cài ignition-fortress)'
    code, out = run(['ign', 'fuel', 'download', '-u', uri, '-v', '2'], timeout)
    if code is None:
        return False, 'quá {} s'.format(timeout)
    if cached_model_dir(uri):
        return True, ''
    tail = ' '.join(out.strip().splitlines()[-2:]) if out else ''
    return False, 'mã {} {}'.format(code, tail)


def pick_runner():
    try:
        from ament_index_python.packages import get_package_share_directory
    except ImportError:
        return 'ros_gz_sim'
    for runner in ('ros_gz_sim', 'ros_ign_gazebo'):
        try:
            get_package_share_directory(runner)
            return runner
        except Exception:  # noqa: BLE001
            continue
    return None


def spawn(world, obj, runner, timeout):
    code, out = run(create_command(world, obj, runner), timeout)
    text = out or ''
    if 'already exists' in text:
        return True, 'đã có sẵn trong world'
    if code == 0:            # create trả 0 khi Gazebo đã nhận yêu cầu tạo entity
        return True, ''
    if code is None:
        return False, 'quá {} s'.format(timeout)
    lines = [ln for ln in text.strip().splitlines() if ln.strip()]
    return False, 'mã {}: {}'.format(code, lines[-1] if lines else '')


def main():
    ap = argparse.ArgumentParser(description='Spawn vật thể từ config/objects.yaml vào Gazebo')
    ap.add_argument('--objects', help='file objects.yaml (mặc định config/objects.yaml)')
    ap.add_argument('--world', help='tên world Gazebo (mặc định: khoá "world" trong file, maze)')
    ap.add_argument('--download-only', action='store_true', help='chỉ tải model về cache rồi thoát')
    ap.add_argument('--list', action='store_true', help='chỉ in danh sách + trạng thái cache')
    ap.add_argument('--download-timeout', type=float, default=180.0)
    ap.add_argument('--spawn-timeout', type=float, default=60.0)
    ap.add_argument('--delay', type=float, default=0.0, help='chờ N giây trước khi spawn')
    args, _ros_args = ap.parse_known_args()          # bỏ qua --ros-args ... khi chạy từ launch

    path = args.objects or lc.find_config('config', 'objects.yaml')[0]
    if not path or not os.path.isfile(path):
        print('[spawn_objects] Không tìm thấy file vật thể: {}'.format(path or 'config/objects.yaml'),
              flush=True)
        return 2
    try:
        world, objects = load_objects_file(path)
    except (OSError, ValueError, yaml.YAMLError) as exc:
        print('[spawn_objects] LỖI: {}'.format(exc), flush=True)
        return 2
    world = args.world or world
    print('[spawn_objects] {} vật thể từ {} -> world "{}"'.format(len(objects), path, world), flush=True)

    status = {}
    for o in objects:
        cached = cached_model_dir(o['uri'])
        status[o['name']] = cached
        if args.list:
            print('  {:<22} ({:6.2f}, {:6.2f})  {}  [{}]'.format(
                o['name'], float(o['x']), float(o['y']), o['uri'],
                'có trong cache' if cached else 'CHƯA tải'), flush=True)
    if args.list:
        return 0

    # 1. tải model còn thiếu
    missing = [o for o in objects if not status[o['name']]]
    offline = False
    for o in missing:
        if offline:
            print('[spawn_objects]   bỏ qua tải "{}" (có vẻ không có mạng)'.format(o['name']), flush=True)
            continue
        print('[spawn_objects] Tải model "{}" từ Gazebo Fuel ...'.format(o['uri']), flush=True)
        t0 = time.monotonic()
        ok, why = download(o['uri'], args.download_timeout)
        if ok:
            status[o['name']] = cached_model_dir(o['uri'])
            print('[spawn_objects]   xong sau {:.0f} s'.format(time.monotonic() - t0), flush=True)
        else:
            print('[spawn_objects]   KHÔNG tải được ({}). Kiểm tra Internet rồi chạy lại: '
                  'ros2 run tb4_lab04 spawn_objects.py --download-only'.format(why), flush=True)
            # Lỗi rất nhanh (DNS/kết nối) hoặc quá thời gian -> gần như chắc chắn mất mạng: không thử
            # tiếp từng model (mỗi lần có thể chờ tới --download-timeout).
            if time.monotonic() - t0 < 30.0 or why.startswith('quá'):
                offline = True
    if args.download_only:
        n = sum(1 for o in objects if status[o['name']])
        print('[spawn_objects] Cache: {}/{} model sẵn sàng.'.format(n, len(objects)), flush=True)
        return 0 if n == len(objects) else 1

    # 2. spawn
    runner = pick_runner()
    if runner is None:
        print('[spawn_objects] Không có ros_gz_sim / ros_ign_gazebo -> sudo apt install '
              'ros-humble-ros-gz-sim', flush=True)
        return 2
    if args.delay > 0:
        time.sleep(args.delay)
    done = 0
    for o in objects:
        if not status[o['name']]:
            print('[spawn_objects] Bỏ qua {} (chưa có model trong cache)'.format(o['name']), flush=True)
            continue
        ok, why = spawn(world, o, runner, args.spawn_timeout)
        if ok:
            done += 1
            print('[spawn_objects] + {:<22} tại ({:.2f}, {:.2f}){}'.format(
                o['name'], float(o['x']), float(o['y']), ' – ' + why if why else ''), flush=True)
        else:
            print('[spawn_objects] ! Spawn {} thất bại ({})'.format(o['name'], why), flush=True)
    print('[spawn_objects] Đã thêm {}/{} vật thể vào world "{}".'.format(done, len(objects), world),
          flush=True)
    return 0 if done == len(objects) else 1


if __name__ == '__main__':
    lc.install_ctrl_c_handlers()
    sys.exit(lc.run_main(main, interrupted_code=0))
