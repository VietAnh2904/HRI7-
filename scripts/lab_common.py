"""Tiện ích dùng chung cho các script mới (điều hướng tự động + YOLO) của tb4_lab04.

Không phải node – được các script trong cùng thư mục `import lab_common` (Python tự thêm thư mục chứa
script vào sys.path, kể cả khi chạy qua `ros2 run` hay khi colcon dùng --symlink-install).

Gồm:
  * xử lý Ctrl+C êm (không traceback, không log đỏ "process has died") – cùng cách với các script cũ;
  * tìm file cấu hình/bản đồ trong thư mục SOURCE của package (sửa xong không cần colcon build lại);
  * chuyển đổi yaw <-> quaternion, chuẩn hoá góc;
  * thư mục lưu kết quả (~/tb4_results/...).
Phần thuần Python (không import ROS) để unit test chạy được cả khi không có ROS.
"""

import math
import os
import signal
import sys
import threading
import time

PKG = 'tb4_lab04'
SIGINT_STATE = {'got': False}


# ------------------------------------------------------------------ Ctrl+C
def _on_sigint(signum, frame):
    SIGINT_STATE['got'] = True
    raise KeyboardInterrupt


def install_ctrl_c_handlers():
    """Ctrl+C trong terminal + launch gửi SIGINT 2 lần, có khi rơi đúng lúc rclpy đang khởi tạo. Chỉ nuốt
    các thông báo rác phát sinh SAU khi người dùng đã bấm Ctrl+C; lỗi thật trước đó vẫn được in."""
    signal.signal(signal.SIGINT, _on_sigint)
    old_unraisable, old_except, old_thread = sys.unraisablehook, sys.excepthook, threading.excepthook

    def unraisable(u):
        if not SIGINT_STATE['got']:
            old_unraisable(u)

    def except_hook(tp, val, tb):
        if issubclass(tp, KeyboardInterrupt) or (SIGINT_STATE['got'] and issubclass(tp, Exception)):
            print('[tb4_lab04] Đã dừng (Ctrl+C).', flush=True)
            return
        old_except(tp, val, tb)

    def thread_hook(args):
        if issubclass(args.exc_type, KeyboardInterrupt) or SIGINT_STATE['got']:
            return
        old_thread(args)

    sys.unraisablehook, sys.excepthook, threading.excepthook = unraisable, except_hook, thread_hook


def ros_shutting_down():
    """True nếu đang tắt vì Ctrl+C hoặc context rclpy đã bị tắt (lỗi lúc đó là race khi tắt)."""
    if SIGINT_STATE['got']:
        return True
    try:
        from rclpy.utilities import get_default_context
        ctx = get_default_context()
        return ctx.handle is not None and not ctx.ok()
    except Exception:  # noqa: BLE001
        return False


def run_main(main, interrupted_code=130):
    """Gọi main() và trả mã thoát; Ctrl+C ở bất kỳ lúc nào -> thoát êm, không traceback."""
    try:
        code = main()
        return 0 if code is None else int(code)
    except KeyboardInterrupt:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        print('[tb4_lab04] Đã dừng (Ctrl+C).', flush=True)
        return interrupted_code
    except Exception as exc:  # noqa: BLE001
        if type(exc).__name__ != 'ExternalShutdownException' and not ros_shutting_down():
            raise
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        return interrupted_code


# ------------------------------------------------------------------ đường dẫn
def package_share_dir():
    try:
        from ament_index_python.packages import get_package_share_directory
        return get_package_share_directory(PKG)
    except Exception:  # noqa: BLE001 (chưa source workspace / chạy unit test)
        return None


def package_source_dir():
    """Thư mục source của package (CMake ghi vào share/tb4_lab04/source_dir.txt lúc build). Khi chạy
    trực tiếp từ repo (`python3 scripts/x.py`) thì là thư mục cha của scripts/."""
    share = package_share_dir()
    if share:
        try:
            with open(os.path.join(share, 'source_dir.txt'), 'r', encoding='utf-8') as f:
                path = f.read().strip()
            if path and os.path.isdir(path):
                return path
        except OSError:
            pass
    here = os.path.dirname(os.path.realpath(__file__))
    if os.path.basename(here) == 'scripts' and os.path.isfile(os.path.join(here, '..', 'package.xml')):
        return os.path.dirname(here)
    return None


def find_config(*relative):
    """Đường dẫn file cấu hình: ưu tiên bản trong SOURCE (vừa sửa), sau đó bản đã install."""
    tried = []
    for base in (package_source_dir(), package_share_dir()):
        if base:
            path = os.path.join(base, *relative)
            tried.append(path)
            if os.path.isfile(path):
                return path, tried
    return None, tried


def results_dir(*sub):
    """~/tb4_results/<sub...> (tạo nếu chưa có). Đặt TB4_RESULTS_DIR để đổi thư mục gốc."""
    root = os.path.expanduser(os.environ.get('TB4_RESULTS_DIR', '~/tb4_results'))
    path = os.path.join(root, *sub)
    os.makedirs(path, exist_ok=True)
    return path


def timestamp():
    return time.strftime('%Y%m%d_%H%M%S')


# ------------------------------------------------------------------ góc
def normalize_angle(a):
    """Đưa góc về (-pi, pi]."""
    a = math.fmod(a + math.pi, 2.0 * math.pi)
    if a <= 0.0:
        a += 2.0 * math.pi
    return a - math.pi


def quat_from_yaw(yaw):
    """(x, y, z, w) của phép quay quanh trục z."""
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def yaw_from_quat(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def stamp_to_sec(stamp):
    return stamp.sec + stamp.nanosec * 1e-9
