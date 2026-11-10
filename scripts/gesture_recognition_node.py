#!/usr/bin/env python3
"""Bài 6 – Phần 2: node nhận diện cử chỉ tay từ webcam bằng MediaPipe Gesture Recognizer.

  ros2 run tb4_lab04 gesture_recognition_node.py
  ros2 launch tb4_lab04 gesture.launch.py                         # + robot_behavior_node
  ros2 run tb4_lab04 gesture_recognition_node.py --ros-args -p camera_source:=1      # webcam USB
  ros2 run tb4_lab04 gesture_recognition_node.py --ros-args -p camera_source:=~/thu.mp4  # video thử

Luồng xử lý mỗi khung hình:
  webcam (OpenCV VideoCapture, 640x480) -> lật gương -> MediaPipe Gesture Recognizer (chế độ VIDEO:
  phát hiện lòng bàn tay -> 21 landmark -> phân loại 7 cử chỉ + None, kèm độ tin cậy)
  -> WaveDetector (vẫy tay = bàn tay xoè dao động ngang, model gốc không có lớp này)
  -> nhãn khung (độ tin cậy < min_score -> None)
  -> GestureStabilizer (bỏ phiếu trong cửa sổ thời gian -> chống nhiễu)
  -> publish /detected_gesture (std_msgs/String, tên cử chỉ ổn định, MỖI khung – đồng thời là
     "nhịp tim" cho watchdog của robot_behavior_node)
  -> cửa sổ OpenCV: landmark, tên cử chỉ + độ tin cậy, nhãn ổn định, trạng thái robot.

Topic:
  pub  /detected_gesture        std_msgs/String  'Thumb_Up' | 'Open_Palm' | 'Wave' | ... | 'None'
  pub  /detected_gesture/info   std_msgs/String  JSON: raw, score, stable, fps, wave_swings
  pub  /gesture/image           sensor_msgs/Image (bgr8, chỉ khi publish_image:=true)
  sub  /robot_behavior/state    std_msgs/String  JSON trạng thái robot (để vẽ lên cửa sổ)
Cử chỉ: Closed_Fist, Open_Palm, Pointing_Up, Thumb_Down, Thumb_Up, Victory, ILoveYou (MediaPipe) + Wave.

Không thấy camera / đọc lỗi -> KHÔNG publish (robot_behavior_node mất tín hiệu -> robot dừng) và thử
mở lại mỗi 2 s. Phím q / Esc trên cửa sổ chỉ để đóng node (không điều khiển robot).
"""

import hashlib
import json
import os
import signal
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
import gesture_logic as gl  # noqa: E402

WINDOW = 'Gesture Recognition - TurtleBot 4'
MODEL_FILE = 'gesture_recognizer.task'
MODEL_URL = ('https://storage.googleapis.com/mediapipe-models/gesture_recognizer/'
             'gesture_recognizer/float16/1/gesture_recognizer.task')
MODEL_SHA256 = '97952348cf6a6a4915c2ea1496b4b37ebabc50cbbf80571435643c455f2b0482'
IMAGE_EXT = ('.jpg', '.jpeg', '.png', '.bmp')
VIDEO_EXT = ('.mp4', '.avi', '.mkv', '.mov', '.webm', '.m4v')
INSTALL_HINT = ('Cài thư viện (Ubuntu 22.04 + ROS 2 Humble, KHÔNG dùng conda):\n'
                '    bash scripts/install_gesture_deps.sh\n'
                'hoặc:\n'
                '    pip3 install --user "mediapipe==0.10.21" "numpy<2" '
                '"opencv-python<4.11" "opencv-contrib-python<4.11"')

COLORS = {  # BGR
    'Open_Palm': (60, 60, 255), 'Closed_Fist': (0, 140, 255), 'Thumb_Up': (60, 200, 60),
    'Pointing_Up': (255, 180, 0), 'Victory': (255, 100, 200), 'Wave': (0, 220, 255),
    'Thumb_Down': (160, 160, 160), 'ILoveYou': (200, 120, 255), 'None': (180, 180, 180),
}
LEGEND = [
    ('Wave', 'chon: chao (xoay trai-phai)'),
    ('Pointing_Up', 'chon: tien-lui 4 lan'),
    ('Victory', 'chon: quy dao tron'),
    ('Thumb_Up', 'XAC NHAN -> thuc hien'),
    ('Open_Palm', 'DUNG ngay'),
    ('Closed_Fist', 'HUY -> cho'),
]


# ------------------------------------------------------------------ tiện ích thuần (có unit test)
def parse_camera_source(value):
    """'0' / 0 -> 0 (chỉ số webcam); '/dev/video2' / 'video.mp4' / thư mục ảnh -> chuỗi (đã expanduser)."""
    if isinstance(value, int):
        return value
    text = str(value).strip()
    if text.lstrip('-').isdigit():
        return int(text)
    return os.path.expanduser(text)


def parse_window_position(text):
    """'1000,40' -> (1000, 40); rỗng / sai định dạng -> None (để hệ điều hành tự đặt)."""
    try:
        x, y = [int(float(v)) for v in str(text).split(',')]
        return x, y
    except (TypeError, ValueError):
        return None


def sha256_of(path):
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        for chunk in iter(lambda: f.read(1 << 20), b''):
            h.update(chunk)
    return h.hexdigest()


def candidate_model_paths():
    """Thứ tự tìm model: models/ trong source -> models/ đã install -> ~/.cache/tb4_lab04/models."""
    out = []
    try:
        import lab_common as lc
        for base in (lc.package_source_dir(), lc.package_share_dir()):
            if base:
                out.append(os.path.join(base, 'models', MODEL_FILE))
    except Exception:  # noqa: BLE001
        pass
    here = os.path.dirname(os.path.realpath(__file__))
    out.append(os.path.join(os.path.dirname(here), 'models', MODEL_FILE))
    out.append(os.path.join(os.path.expanduser('~/.cache/tb4_lab04/models'), MODEL_FILE))
    seen, uniq = set(), []
    for p in out:
        p = os.path.abspath(p)
        if p not in seen:
            seen.add(p)
            uniq.append(p)
    return uniq


def download_model(url, dest, expected_sha=None, timeout=60, log=print):
    """Tải model về file tạm rồi đổi tên (không để lại file hỏng nếu đứt mạng)."""
    import urllib.request
    os.makedirs(os.path.dirname(dest), exist_ok=True)
    tmp = dest + '.part'
    log('Tải model MediaPipe: {} -> {}'.format(url, dest))
    try:
        with urllib.request.urlopen(url, timeout=timeout) as resp, open(tmp, 'wb') as f:
            while True:
                chunk = resp.read(1 << 16)
                if not chunk:
                    break
                f.write(chunk)
    except BaseException:
        if os.path.exists(tmp):
            os.remove(tmp)          # không để lại file tải dở
        raise
    if os.path.getsize(tmp) < 100000:
        os.remove(tmp)
        raise RuntimeError('file tải về quá nhỏ – có thể là trang lỗi chứ không phải model')
    digest = sha256_of(tmp)
    if expected_sha and digest != expected_sha:
        log('CẢNH BÁO: SHA256 của model khác bản đã kiểm thử ({}...). Vẫn dùng – nếu lỗi, xoá {} '
            'và tải lại.'.format(digest[:12], dest))
    os.replace(tmp, dest)
    return dest


def resolve_model(model_path, url, expected_sha, log=print):
    if model_path:
        p = os.path.abspath(os.path.expanduser(model_path))
        if not os.path.isfile(p):
            raise FileNotFoundError('model_path không tồn tại: {}'.format(p))
        return p
    cands = candidate_model_paths()
    for p in cands:
        if os.path.isfile(p) and os.path.getsize(p) > 100000:
            return p
    return download_model(url, cands[-1], expected_sha, log=log)


def list_images(folder):
    return sorted(os.path.join(folder, f) for f in os.listdir(folder)
                  if f.lower().endswith(IMAGE_EXT))


# ------------------------------------------------------------------ nguồn ảnh
class FrameSource:
    """Webcam (luồng đọc riêng, luôn lấy khung MỚI NHẤT), file video (đúng nhịp fps) hoặc thư mục ảnh."""

    def __init__(self, cv2, source, width, height, fps, loop, image_hold, log, fourcc=''):
        self.fourcc = (fourcc or '').strip().upper()
        self.cv2, self.source, self.width, self.height = cv2, source, width, height
        self.fps, self.loop, self.image_hold, self.log = fps, loop, image_hold, log
        self.kind = 'camera'
        if isinstance(source, str):
            if os.path.isdir(source):
                self.kind = 'images'
            elif source.lower().endswith(VIDEO_EXT) and os.path.isfile(source):
                self.kind = 'video'
            elif not source.startswith('/dev/'):
                self.kind = 'video'      # URL/pipeline… để OpenCV tự xử lý
        self.cap = None
        self.lock = threading.Lock()
        self.latest = None
        self.latest_id = 0
        self.read_id = 0
        self.running = False
        self.thread = None
        self.finished = False
        self.images = []
        self.img_index = 0
        self.img_t0 = None
        self.video_period = None
        self.next_video_t = None
        self.fail_count = 0
        self.last_open_try = 0.0
        self.opened_once = False

    # -- webcam
    def _open_camera(self):
        cv2 = self.cv2
        src = self.source
        cap = None
        if sys.platform.startswith('linux'):
            cap = cv2.VideoCapture(src, cv2.CAP_V4L2)
            if not cap.isOpened():
                cap.release()
                cap = None
        if cap is None:
            cap = cv2.VideoCapture(src)
        if not cap.isOpened():
            cap.release()
            return None
        if len(self.fourcc) == 4:       # vd MJPG: nhiều webcam chỉ đạt 30 FPS ở chế độ nén MJPG
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*self.fourcc))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        if self.fps > 0:
            cap.set(cv2.CAP_PROP_FPS, self.fps)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        return cap

    def _camera_loop(self):
        while self.running:
            if self.cap is None:
                now = time.monotonic()
                if now - self.last_open_try < 2.0:
                    time.sleep(0.05)
                    continue
                self.last_open_try = now
                self.cap = self._open_camera()
                if self.cap is None:
                    self.log('warn', 'Không mở được camera {} – thử lại sau 2 s. Gợi ý: `ls /dev/video*`, '
                                     'đổi camera_source:=1, thêm user vào nhóm video '
                                     '(sudo usermod -aG video $USER), đóng app khác đang dùng webcam; '
                                     'máy ảo/WSL2 phải chuyển webcam vào máy ảo.'.format(self.source))
                    continue
                w = int(self.cap.get(self.cv2.CAP_PROP_FRAME_WIDTH))
                h = int(self.cap.get(self.cv2.CAP_PROP_FRAME_HEIGHT))
                f = self.cap.get(self.cv2.CAP_PROP_FPS)
                self.log('info', 'Đã mở camera {} ({}x{}, {:.0f} FPS danh định)'.format(self.source, w, h, f or 0))
                self.opened_once = True
                self.fail_count = 0
            ok, frame = self.cap.read()
            if not ok or frame is None:
                self.fail_count += 1
                if self.fail_count >= 30:
                    self.log('warn', 'Camera không trả khung hình – đóng và mở lại')
                    self.cap.release()
                    self.cap = None
                    with self.lock:
                        self.latest = None
                time.sleep(0.01)
                continue
            self.fail_count = 0
            with self.lock:
                self.latest = frame
                self.latest_id += 1

    # -- chung
    def start(self):
        if self.kind == 'camera':
            self.running = True
            self.thread = threading.Thread(target=self._camera_loop, daemon=True)
            self.thread.start()
        elif self.kind == 'images':
            self.images = list_images(self.source)
            if not self.images:
                raise RuntimeError('Thư mục {} không có ảnh ({})'.format(self.source, ', '.join(IMAGE_EXT)))
            self.log('info', 'Nguồn ảnh: {} ảnh trong {} ({} s/ảnh)'.format(
                len(self.images), self.source, self.image_hold))
        else:
            self.cap = self.cv2.VideoCapture(self.source)
            if not self.cap.isOpened():
                raise RuntimeError('Không mở được video {}'.format(self.source))
            fps = self.cap.get(self.cv2.CAP_PROP_FPS) or 0.0
            self.video_period = 1.0 / fps if 1.0 <= fps <= 240.0 else 1.0 / 30.0
            self.log('info', 'Nguồn video: {} ({:.1f} fps)'.format(self.source, 1.0 / self.video_period))

    def read(self, timeout=0.5):
        """Trả khung BGR mới hoặc None (chưa có khung / camera lỗi / hết video)."""
        if self.kind == 'camera':
            t_end = time.monotonic() + timeout
            while time.monotonic() < t_end:
                with self.lock:
                    if self.latest is not None and self.latest_id != self.read_id:
                        self.read_id = self.latest_id
                        return self.latest.copy()
                time.sleep(0.003)
            return None
        if self.kind == 'images':
            now = time.monotonic()
            if self.img_t0 is None:
                self.img_t0 = now
            idx = int((now - self.img_t0) / max(self.image_hold, 0.05))
            if idx >= len(self.images):
                if not self.loop:
                    self.finished = True
                    return None
                self.img_t0, idx = now, 0
            img = self.cv2.imread(self.images[idx])
            time.sleep(1.0 / 30.0)
            return img
        # video: phát đúng nhịp thời gian thực (để bộ lọc thời gian hoạt động như với webcam)
        now = time.monotonic()
        if self.next_video_t is None:
            self.next_video_t = now
        if now < self.next_video_t:
            time.sleep(self.next_video_t - now)
        self.next_video_t += self.video_period
        if time.monotonic() - self.next_video_t > 0.5:      # xử lý chậm hơn video: bỏ khung cũ
            skip = int((time.monotonic() - self.next_video_t) / self.video_period)
            for _ in range(skip):
                self.cap.grab()
            self.next_video_t += skip * self.video_period
        ok, frame = self.cap.read()
        if not ok:
            if self.loop:
                self.cap.set(self.cv2.CAP_PROP_POS_FRAMES, 0)
                ok, frame = self.cap.read()
            if not ok:
                self.finished = True
                return None
        return frame

    def healthy(self):
        if self.kind == 'camera':
            return self.cap is not None and self.fail_count < 30
        return not self.finished

    def stop(self):
        self.running = False
        if self.thread is not None:
            self.thread.join(timeout=1.0)
        if self.cap is not None:
            try:
                self.cap.release()
            except Exception:  # noqa: BLE001
                pass


# ------------------------------------------------------------------ vẽ
def draw_overlay(cv2, img, landmarks, raw_label, raw_score, stable, stable_ratio, fps, robot, wave_swings,
                 min_swings, source_ok=True):
    h, w = img.shape[:2]
    if landmarks:
        pts = [(int(x * w), int(y * h)) for x, y in landmarks]
        for a, b in gl.HAND_CONNECTIONS:
            cv2.line(img, pts[a], pts[b], (230, 230, 230), 2, cv2.LINE_AA)
        for i, p in enumerate(pts):
            cv2.circle(img, p, 4 if i not in gl.FINGER_TIPS else 6, (0, 0, 255) if i else (255, 0, 0), -1,
                       cv2.LINE_AA)
    font = cv2.FONT_HERSHEY_SIMPLEX
    # khung trên
    cv2.rectangle(img, (0, 0), (w, 92), (25, 25, 25), -1)
    col = COLORS.get(raw_label, (255, 255, 255))
    txt = 'Khung: {} {:.0f}%'.format(gl.DISPLAY_NAME.get(raw_label, raw_label), 100 * raw_score) \
        if raw_label != gl.NONE else 'Khung: (khong co cu chi)'
    cv2.putText(img, txt, (10, 28), font, 0.7, col, 2, cv2.LINE_AA)
    scol = COLORS.get(stable, (255, 255, 255))
    cv2.putText(img, 'ON DINH -> /detected_gesture: {}'.format(stable), (10, 58), font, 0.65, scol, 2,
                cv2.LINE_AA)
    info = 'FPS {:.1f}   wave {}/{}'.format(fps, wave_swings, min_swings)
    cv2.putText(img, info, (10, 84), font, 0.5, (200, 200, 200), 1, cv2.LINE_AA)
    # thanh độ tin cậy
    bx = w - 170
    cv2.rectangle(img, (bx, 18), (bx + 150, 34), (90, 90, 90), 1)
    cv2.rectangle(img, (bx, 18), (bx + int(150 * max(0.0, min(1.0, raw_score))), 34), col, -1)
    cv2.putText(img, 'conf', (bx - 48, 32), font, 0.5, (200, 200, 200), 1, cv2.LINE_AA)
    cv2.rectangle(img, (bx, 48), (bx + 150, 64), (90, 90, 90), 1)
    cv2.rectangle(img, (bx, 48), (bx + int(150 * max(0.0, min(1.0, stable_ratio))), 64), scol, -1)
    cv2.putText(img, 'vote', (bx - 48, 62), font, 0.5, (200, 200, 200), 1, cv2.LINE_AA)
    # chú giải (nền tối bán trong suốt để đọc được trên mọi phông nền)
    y0 = 112
    x1 = min(w - 1, 300)
    y1 = min(h - 60, y0 + 20 * len(LEGEND) - 8)
    roi = img[y0 - 16:y1, 0:x1]
    if roi.size:
        img[y0 - 16:y1, 0:x1] = (roi * 0.45).astype(roi.dtype)
    for i, (g, text) in enumerate(LEGEND):
        y = y0 + i * 20
        cv2.putText(img, '{:<12} {}'.format(g, text), (8, y), font, 0.45,
                    COLORS.get(g, (255, 255, 255)), 1, cv2.LINE_AA)
    # khung dưới: trạng thái robot
    cv2.rectangle(img, (0, h - 54), (w, h), (25, 25, 25), -1)
    if robot is None:
        line1, line2, rcol = 'Robot: chua co robot_behavior_node', '', (0, 200, 255)
    else:
        st = robot.get('state', '?')
        sel = robot.get('selected') or '-'
        act = robot.get('active') or '-'
        prog = robot.get('progress')
        line1 = gl.to_ascii('Robot: {}   chon: {}   dang chay: {}{}').format(
            st, sel, act, '' if prog is None else ' ({:.0f}%)'.format(100 * prog))
        line2 = gl.to_ascii(robot.get('message', ''))[:80]
        rcol = {'EXECUTING': (60, 220, 60), 'SELECTED': (0, 220, 255), 'STOPPED': (60, 60, 255),
                'SIGNAL_LOST': (60, 60, 255)}.get(st, (230, 230, 230))
    cv2.putText(img, line1, (10, h - 32), font, 0.55, rcol, 2, cv2.LINE_AA)
    cv2.putText(img, line2, (10, h - 10), font, 0.5, (220, 220, 220), 1, cv2.LINE_AA)
    if not source_ok:
        cv2.putText(img, 'KHONG CO CAMERA - robot se dung (mat tin hieu)', (20, h // 2), font, 0.7,
                    (0, 0, 255), 2, cv2.LINE_AA)
    return img


def image_msg_from_bgr(img, stamp, frame_id, Image):
    msg = Image()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.height, msg.width = int(img.shape[0]), int(img.shape[1])
    msg.encoding = 'bgr8'
    msg.is_bigendian = 0
    msg.step = int(img.shape[1] * 3)
    msg.data = img.tobytes()
    return msg


def fix_qt_fonts():
    """opencv-python (bản pip, giao diện Qt) đặt QT_QPA_FONTDIR = cv2/qt/fonts – thư mục KHÔNG tồn tại ->
    Qt in "QFontDatabase: Cannot find font directory" liên tục. Gọi SAU `import cv2`, TRƯỚC khi mở cửa sổ."""
    cur = os.environ.get('QT_QPA_FONTDIR')
    if cur and os.path.isdir(cur):
        return
    for fdir in ('/usr/share/fonts/truetype/dejavu', '/usr/share/fonts/truetype', '/usr/share/fonts'):
        if os.path.isdir(fdir):
            os.environ['QT_QPA_FONTDIR'] = fdir
            return


# ------------------------------------------------------------------ main
def main(argv=None):
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.signals import SignalHandlerOptions
    from sensor_msgs.msg import Image
    from std_msgs.msg import String

    rclpy.init(args=argv, signal_handler_options=SignalHandlerOptions.NO)
    node = Node('gesture_recognition_node')
    log = node.get_logger()
    stop = {'flag': False, 'count': 0}

    def on_signal(signum, frame):
        stop['count'] += 1
        stop['flag'] = True
        if stop['count'] >= 2:
            raise KeyboardInterrupt

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    from rcl_interfaces.msg import ParameterDescriptor

    def P(name, default):
        # dynamic_typing: "0" hay 0, 30 hay 30.0 trong YAML / dòng lệnh đều được
        node.declare_parameter(name, default, ParameterDescriptor(dynamic_typing=True))
        value = node.get_parameter(name).value
        return default if value is None else value

    prm = {
        'camera_source': str(P('camera_source', '0')),
        'frame_width': int(P('frame_width', 640)),
        'frame_height': int(P('frame_height', 480)),
        'camera_fps': int(P('camera_fps', 30)),
        'camera_fourcc': str(P('camera_fourcc', '')),
        'mirror': gl.as_bool(P('mirror', True)),
        'loop': gl.as_bool(P('loop', False)),
        'image_hold': float(P('image_hold', 2.0)),
        'model_path': str(P('model_path', '')),
        'model_url': str(P('model_url', MODEL_URL)),
        'num_hands': int(P('num_hands', 1)),
        'min_hand_detection_confidence': float(P('min_hand_detection_confidence', 0.5)),
        'min_hand_presence_confidence': float(P('min_hand_presence_confidence', 0.5)),
        'min_tracking_confidence': float(P('min_tracking_confidence', 0.5)),
        'min_score': float(P('min_score', 0.55)),
        'hold_time': float(P('hold_time', 0.5)),
        'stop_hold_time': float(P('stop_hold_time', 0.2)),
        'none_hold_time': float(P('none_hold_time', 0.4)),
        'wave_hold_time': float(P('wave_hold_time', 0.15)),
        'vote_ratio': float(P('vote_ratio', 0.7)),
        'min_frames': int(P('min_frames', 2)),
        'wave_enabled': gl.as_bool(P('wave_enabled', True)),
        'wave_window': float(P('wave_window', 1.5)),
        'wave_min_swings': int(P('wave_min_swings', 3)),
        'wave_min_amplitude': float(P('wave_min_amplitude', 0.06)),
        'wave_latch': float(P('wave_latch', 1.0)),
        'max_fps': float(P('max_fps', 30.0)),
        'show_window': gl.as_bool(P('show_window', True)),
        'window_scale': float(P('window_scale', 1.0)),
        'window_position': str(P('window_position', '')),
        'publish_image': gl.as_bool(P('publish_image', False)),
        'gesture_topic': str(P('gesture_topic', '/detected_gesture')),
        'state_topic': str(P('state_topic', '/robot_behavior/state')),
        'log_changes': gl.as_bool(P('log_changes', True)),
    }

    try:
        import cv2
        import numpy  # noqa: F401
    except Exception as exc:  # noqa: BLE001
        log.fatal('Không import được OpenCV/NumPy: {}\n{}'.format(exc, INSTALL_HINT))
        node.destroy_node()
        rclpy.shutdown()
        return 1
    fix_qt_fonts()
    try:
        import mediapipe as mp
        from mediapipe.tasks.python import BaseOptions
        from mediapipe.tasks.python import vision
    except Exception as exc:  # noqa: BLE001
        log.fatal('Không import được MediaPipe: {}: {}\n{}'.format(type(exc).__name__, exc, INSTALL_HINT))
        node.destroy_node()
        rclpy.shutdown()
        return 1

    def slog(level, msg):
        getattr(log, level)(msg)

    try:
        model_path = resolve_model(prm['model_path'], prm['model_url'], MODEL_SHA256,
                                   log=lambda m: log.info(m))
    except Exception as exc:  # noqa: BLE001
        log.fatal('Không có model {}: {}\nTải tay: wget -O ~/.cache/tb4_lab04/models/{} {}'.format(
            MODEL_FILE, exc, MODEL_FILE, MODEL_URL))
        node.destroy_node()
        rclpy.shutdown()
        return 1

    try:
        options = vision.GestureRecognizerOptions(
            base_options=BaseOptions(model_asset_path=model_path),
            running_mode=vision.RunningMode.VIDEO,
            num_hands=max(1, prm['num_hands']),
            min_hand_detection_confidence=prm['min_hand_detection_confidence'],
            min_hand_presence_confidence=prm['min_hand_presence_confidence'],
            min_tracking_confidence=prm['min_tracking_confidence'])
        recognizer = vision.GestureRecognizer.create_from_options(options)
    except Exception as exc:  # noqa: BLE001
        log.fatal('Không khởi tạo được MediaPipe Gesture Recognizer với {}: {}\n'
                  'Model hỏng? Xoá file đó rồi chạy lại để tải mới.'.format(model_path, exc))
        node.destroy_node()
        rclpy.shutdown()
        return 1
    log.info('MediaPipe {} – model {}'.format(getattr(mp, '__version__', '?'), model_path))

    pub = node.create_publisher(String, prm['gesture_topic'], 10)
    pub_info = node.create_publisher(String, prm['gesture_topic'] + '/info', 10)
    pub_img = node.create_publisher(Image, '/gesture/image', 2) if prm['publish_image'] else None
    robot = {'state': None, 't': 0.0}

    def on_state(msg):
        try:
            robot['state'] = json.loads(msg.data)
            robot['t'] = time.monotonic()
        except ValueError:
            pass

    node.create_subscription(String, prm['state_topic'], on_state, 10)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=lambda: _spin(executor, stop), daemon=True)
    spin_thread.start()

    source = FrameSource(cv2, parse_camera_source(prm['camera_source']), prm['frame_width'],
                         prm['frame_height'], prm['camera_fps'], prm['loop'], prm['image_hold'], slog,
                         prm['camera_fourcc'])
    exit_code = 0
    try:
        source.start()
    except Exception as exc:  # noqa: BLE001
        log.fatal(str(exc))
        stop['flag'] = True
        exit_code = 1

    wave = gl.WaveDetector(prm['wave_window'], prm['wave_min_swings'], prm['wave_min_amplitude'],
                           prm['wave_latch'])
    stab = gl.GestureStabilizer(
        hold_time=prm['hold_time'],
        hold_overrides={gl.STOP_GESTURE: prm['stop_hold_time'], gl.NONE: prm['none_hold_time'],
                        gl.WAVE: prm['wave_hold_time']},
        vote_ratio=prm['vote_ratio'], min_frames=prm['min_frames'])

    show = prm['show_window']
    if show:
        try:
            cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(WINDOW, int(prm['frame_width'] * prm['window_scale']),
                             int(prm['frame_height'] * prm['window_scale']))
            pos = parse_window_position(prm['window_position'])
            if pos is not None:
                cv2.moveWindow(WINDOW, pos[0], pos[1])
        except Exception as exc:  # noqa: BLE001
            log.warn('Không mở được cửa sổ OpenCV ({}): chạy không hiển thị. Nếu dùng '
                     'opencv-python-headless: pip3 uninstall -y opencv-python-headless'.format(exc))
            show = False

    log.info('Đang nhận diện cử chỉ -> {} (Ctrl+C để dừng)'.format(prm['gesture_topic']))
    t0 = time.monotonic()
    last_ts_ms = -1
    fps = 0.0
    last_frame_t = None
    min_period = 1.0 / prm['max_fps'] if prm['max_fps'] > 0 else 0.0
    last_proc = 0.0
    last_stable_logged = None
    no_frame_since = None
    frames = 0
    last_report = time.monotonic()
    placeholder = None
    errors = 0
    try:
        while not stop['flag'] and rclpy.ok():
            frame = source.read(timeout=0.25)
            now = time.monotonic()
            if frame is None:
                if source.kind != 'camera' and source.finished:
                    log.info('Hết nguồn ảnh/video -> dừng node (robot sẽ dừng do mất tín hiệu)')
                    break
                if no_frame_since is None:
                    no_frame_since = now
                if show:
                    if placeholder is None:
                        placeholder = numpy_zeros(cv2, prm['frame_height'], prm['frame_width'])
                    img = placeholder.copy()
                    draw_overlay(cv2, img, None, gl.NONE, 0.0, '-', 0.0, 0.0,
                                 robot['state'] if now - robot['t'] < 2.0 else None, 0,
                                 prm['wave_min_swings'], source_ok=False)
                    if _show(cv2, img, show):
                        break
                continue
            no_frame_since = None
            # dung sai 20 %: webcam 30 FPS giao khung cách nhau ~33 ms ± vài ms; so sánh đúng bằng 1/max_fps sẽ
            # bỏ nhầm gần một nửa số khung
            if min_period and now - last_proc < 0.8 * min_period:
                continue
            last_proc = now
            try:
                if frame.ndim == 2:
                    frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
                elif frame.shape[2] == 4:
                    frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
                if prm['mirror']:
                    frame = cv2.flip(frame, 1)
                rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                ts_ms = int((now - t0) * 1000.0)
                if ts_ms <= last_ts_ms:
                    ts_ms = last_ts_ms + 1
                last_ts_ms = ts_ms
                mp_img = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)
                result = recognizer.recognize_for_video(mp_img, ts_ms)
                errors = 0
            except Exception as exc:  # noqa: BLE001 – 1 khung lỗi không làm chết node
                errors += 1
                log.error('Lỗi xử lý khung hình ({}/10): {}: {}'.format(errors, type(exc).__name__, exc))
                if errors >= 10:
                    log.fatal('10 khung liên tiếp lỗi -> dừng node (robot sẽ dừng do mất tín hiệu)')
                    exit_code = 1
                    break
                continue

            landmarks, category, score = None, gl.NONE, 0.0
            if result.hand_landmarks:
                best = 0
                if result.gestures and len(result.gestures) > 1:
                    best = max(range(len(result.gestures)),
                               key=lambda i: result.gestures[i][0].score if result.gestures[i] else 0)
                landmarks = [(lm.x, lm.y) for lm in result.hand_landmarks[best]]
                if result.gestures and len(result.gestures) > best and result.gestures[best]:
                    c = result.gestures[best][0]
                    category, score = gl.canonical_gesture(c.category_name), float(c.score)
            hand = landmarks is not None
            waving = False
            if prm['wave_enabled']:
                is_open = hand and (category == gl.STOP_GESTURE or gl.count_extended_fingers(landmarks) >= 4)
                cx = gl.palm_center(landmarks)[0] if hand else None
                waving = wave.update(now, cx, is_open)
            label = gl.frame_label(category, score, prm['min_score'], hand, waving)
            raw_score = wave.confidence() if label == gl.WAVE else (score if label != gl.NONE else 0.0)
            stable, changed = stab.update(now, label)

            pub.publish(String(data=stable))
            info = {'raw': category, 'score': round(score, 3), 'label': label, 'stable': stable,
                    'vote': round(stab.stable_ratio, 2), 'fps': round(fps, 1),
                    'wave_swings': wave.last_swings, 'hand': hand}
            pub_info.publish(String(data=json.dumps(info)))
            if changed and prm['log_changes'] and stable != last_stable_logged:
                log.info('Cử chỉ ổn định: {:<12} (khung: {} {:.2f})'.format(stable, category, score))
                last_stable_logged = stable

            frames += 1
            if now - last_report >= 30.0 and fps > 0:
                last_report = now
                if fps < 8.0:
                    log.warn('FPS thấp ({:.1f}) – vẫy tay cần >= 8-10 FPS mới nhận được; giảm tải máy hoặc '
                             'max_fps/độ phân giải'.format(fps))
                else:
                    log.info('FPS {:.1f} – đang publish: {}'.format(fps, stable))
            if last_frame_t is not None:
                dt = now - last_frame_t
                if dt > 0:
                    fps = 0.9 * fps + 0.1 * (1.0 / dt) if fps > 0 else 1.0 / dt
            last_frame_t = now

            if show or pub_img is not None:
                draw_overlay(cv2, frame, landmarks, label, raw_score, stable, stab.stable_ratio, fps,
                             robot['state'] if now - robot['t'] < 2.0 else None, wave.last_swings,
                             prm['wave_min_swings'])
                if pub_img is not None:
                    pub_img.publish(image_msg_from_bgr(frame, node.get_clock().now().to_msg(), 'webcam',
                                                       Image))
                if show and _show(cv2, frame, show):
                    break
    except KeyboardInterrupt:
        pass
    except Exception as exc:  # noqa: BLE001
        log.error('Lỗi trong vòng lặp nhận diện: {}: {}'.format(type(exc).__name__, exc))
        exit_code = 1
    finally:
        stop['flag'] = True
        source.stop()
        try:
            recognizer.close()
        except Exception:  # noqa: BLE001
            pass
        if show:
            try:
                cv2.destroyAllWindows()
                cv2.waitKey(1)
            except Exception:  # noqa: BLE001
                pass
        log.info('Đã xử lý {} khung hình. Thoát.'.format(frames))
        spin_thread.join(timeout=1.0)
        executor.shutdown(timeout_sec=1.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return exit_code


def numpy_zeros(cv2, h, w):
    import numpy as np
    return np.zeros((h, w, 3), dtype=np.uint8)


_WINDOW_STATE = {'seen_visible': False}


def _show(cv2, img, show):
    """Hiển thị; trả True nếu người dùng đóng cửa sổ (nút X) hoặc bấm q/Esc."""
    if not show:
        return False
    cv2.imshow(WINDOW, img)
    key = cv2.waitKey(1) & 0xFF
    if key in (27, ord('q')):
        return True
    try:
        visible = cv2.getWindowProperty(WINDOW, cv2.WND_PROP_VISIBLE)
    except Exception:  # noqa: BLE001
        return False
    # Chỉ coi là "đã đóng" khi cửa sổ TỪNG hiện ra (vài backend trả -1/0 ở những khung đầu tiên).
    if visible >= 1:
        _WINDOW_STATE['seen_visible'] = True
        return False
    return _WINDOW_STATE['seen_visible']


def _spin(executor, stop):
    while not stop['flag']:
        try:
            executor.spin_once(timeout_sec=0.1)
        except Exception:  # noqa: BLE001
            if stop['flag']:
                break
            time.sleep(0.1)


if __name__ == '__main__':
    sys.exit(main())
