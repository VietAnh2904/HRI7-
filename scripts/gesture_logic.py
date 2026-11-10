"""Logic thuần Python của Bài 6 (giao tiếp phi ngôn ngữ bằng cử chỉ tay) – KHÔNG import ROS, OpenCV hay
MediaPipe, nên unit test chạy được ở bất kỳ máy nào (test/test_gesture_logic.py).

Hai node dùng module này:
  * gesture_recognition_node.py : WaveDetector, GestureStabilizer, count_extended_fingers, ...
  * robot_behavior_node.py      : BehaviorStateMachine, build_plan, SegmentRunner, ...

Mọi thời gian tính bằng giây (đồng hồ đơn điệu – time.monotonic()), khoảng cách mét, góc radian.
"""

import json
import math

# ----------------------------------------------------------------------------------------------------
# Tên cử chỉ
# ----------------------------------------------------------------------------------------------------
# 7 lớp của model MediaPipe Gesture Recognizer (canned gestures) + "None". "Wave" KHÔNG có trong model:
# node nhận diện tự suy ra từ chuyển động ngang của bàn tay đang xoè (WaveDetector bên dưới).
MP_GESTURES = ('None', 'Closed_Fist', 'Open_Palm', 'Pointing_Up', 'Thumb_Down', 'Thumb_Up', 'Victory',
               'ILoveYou')
WAVE = 'Wave'
NONE = 'None'
ALL_GESTURES = MP_GESTURES + (WAVE,)

# Nhãn ASCII cho cửa sổ OpenCV (cv2.putText không vẽ được chữ tiếng Việt có dấu).
DISPLAY_NAME = {
    'None': 'None',
    'Closed_Fist': 'Closed Fist (nam tay)',
    'Open_Palm': 'Open Palm (xoe tay)',
    'Pointing_Up': 'Pointing Up (1 ngon)',
    'Thumb_Down': 'Thumb Down',
    'Thumb_Up': 'Thumb Up (ngon cai)',
    'Victory': 'Victory (2 ngon)',
    'ILoveYou': 'I Love You',
    'Wave': 'Wave (vay tay)',
}

# Ánh xạ cử chỉ -> hành vi (bảng của đề bài).
SELECT_GESTURES = {'Pointing_Up': 'forward_back', 'Victory': 'circle', 'Wave': 'greet'}
CONFIRM_GESTURE = 'Thumb_Up'
STOP_GESTURE = 'Open_Palm'
CANCEL_GESTURE = 'Closed_Fist'

BEHAVIOR_TEXT = {
    'greet': 'chao lai (xoay trai-phai)',
    'forward_back': 'tien-lui 4 lan',
    'circle': 'quy dao tron',
}


def to_ascii(text):
    """Bỏ dấu tiếng Việt ('Đã ra giữa sân' -> 'Da ra giua san') – cv2.putText chỉ vẽ được ASCII."""
    import unicodedata
    text = str(text).replace('đ', 'd').replace('Đ', 'D').replace('–', '-').replace('—', '-')
    text = text.replace('→', '->').replace('°', ' deg').replace('≥', '>=').replace('≤', '<=')
    text = unicodedata.normalize('NFD', text)
    text = ''.join(ch for ch in text if unicodedata.category(ch) != 'Mn')
    return text.encode('ascii', 'replace').decode('ascii')


def as_bool(value):
    """Giá trị tham số -> bool ('false', '0', 'no', 'off', '' -> False). bool('false') của Python là True!"""
    if isinstance(value, str):
        return value.strip().lower() in ('1', 'true', 'yes', 'on')
    return bool(value)


def canonical_gesture(name):
    """Chuẩn hoá tên cử chỉ: 'thumb_up', 'Thumb Up', 'THUMB-UP' -> 'Thumb_Up'; lạ/rỗng -> 'None'."""
    if name is None:
        return NONE
    key = str(name).strip().replace('-', '_').replace(' ', '_').lower()
    if not key:
        return NONE
    for g in ALL_GESTURES:
        if g.lower() == key:
            return g
    return NONE


def parse_gesture_message(text):
    """Nội dung std_msgs/String trên /detected_gesture -> tên cử chỉ chuẩn.

    Node nhận diện gửi tên trần ('Thumb_Up'); chấp nhận thêm dạng 'Thumb_Up 0.87' hoặc JSON
    {"gesture": "Thumb_Up", ...} để có thể thử bằng `ros2 topic pub` cho tiện."""
    text = (text or '').strip()
    if text.startswith('{'):
        try:
            data = json.loads(text)
            return canonical_gesture(data.get('gesture'))
        except (ValueError, AttributeError):
            return NONE
    return canonical_gesture(text.split()[0] if text else '')


# ----------------------------------------------------------------------------------------------------
# Hình học bàn tay (21 landmark của MediaPipe, toạ độ chuẩn hoá 0..1)
# ----------------------------------------------------------------------------------------------------
WRIST = 0
FINGER_TIPS = (8, 12, 16, 20)      # ngón trỏ, giữa, áp út, út
FINGER_PIPS = (6, 10, 14, 18)
PALM_POINTS = (0, 5, 9, 13, 17)
HAND_CONNECTIONS = (
    (0, 1), (1, 2), (2, 3), (3, 4),            # ngón cái
    (0, 5), (5, 6), (6, 7), (7, 8),            # trỏ
    (5, 9), (9, 10), (10, 11), (11, 12),       # giữa
    (9, 13), (13, 14), (14, 15), (15, 16),     # áp út
    (13, 17), (17, 18), (18, 19), (19, 20),    # út
    (0, 17),
)


def palm_center(landmarks):
    """Tâm lòng bàn tay (trung bình cổ tay + 4 gốc ngón) – ổn định hơn cổ tay khi vẫy."""
    xs = [landmarks[i][0] for i in PALM_POINTS]
    ys = [landmarks[i][1] for i in PALM_POINTS]
    return sum(xs) / len(xs), sum(ys) / len(ys)


def count_extended_fingers(landmarks, ratio=1.15):
    """Số ngón (trỏ..út) đang duỗi: đầu ngón xa cổ tay hơn khớp PIP ít nhất `ratio` lần.

    Dùng khi vẫy nhanh: ảnh nhoè làm model trả 'None' dù tay vẫn xoè."""
    wx, wy = landmarks[WRIST][0], landmarks[WRIST][1]
    n = 0
    for tip, pip in zip(FINGER_TIPS, FINGER_PIPS):
        d_tip = math.hypot(landmarks[tip][0] - wx, landmarks[tip][1] - wy)
        d_pip = math.hypot(landmarks[pip][0] - wx, landmarks[pip][1] - wy)
        if d_pip > 1e-6 and d_tip >= ratio * d_pip:
            n += 1
    return n


def count_swings(xs, min_amplitude):
    """Số lần đổi chiều (zig-zag) của chuỗi toạ độ x, mỗi nhịp phải dài >= min_amplitude.

    Ví dụ phải-trái-phải đủ biên độ = 3 nhịp. Dao động nhỏ hơn min_amplitude (rung tay, nhiễu landmark)
    không được tính – đó là cơ chế chống nhận nhầm "Wave" khi chỉ giơ bàn tay đứng yên."""
    if len(xs) < 2:
        return 0
    lo = hi = xs[0]
    direction = 0
    extreme = xs[0]
    swings = 0
    for x in xs[1:]:
        if direction == 0:
            lo, hi = min(lo, x), max(hi, x)
            if x - lo >= min_amplitude:
                direction, extreme, swings = 1, x, 1
            elif hi - x >= min_amplitude:
                direction, extreme, swings = -1, x, 1
        elif direction == 1:
            if x > extreme:
                extreme = x
            elif extreme - x >= min_amplitude:
                direction, extreme, swings = -1, x, swings + 1
        else:
            if x < extreme:
                extreme = x
            elif x - extreme >= min_amplitude:
                direction, extreme, swings = 1, x, swings + 1
    return swings


class WaveDetector:
    """Nhận "vẫy tay" = bàn tay XOÈ di chuyển qua lại theo phương ngang.

    - chỉ lấy các khung có bàn tay xoè (Open_Palm hoặc >= 4 ngón duỗi);
    - trong cửa sổ `window` giây phải có >= `min_swings` nhịp, mỗi nhịp dài >= `min_amplitude`
      (tính theo bề ngang ảnh, 0..1);
    - mất tay / nắm tay quá `max_gap` giây -> xoá lịch sử (không ghép hai lần giơ tay rời nhau);
    - sau khi nhận ra, giữ kết quả `latch` giây (để bộ lọc ổn định phía sau kịp xác nhận).
    """

    def __init__(self, window=1.5, min_swings=3, min_amplitude=0.06, latch=1.0, max_gap=0.35):
        self.window = float(window)
        self.min_swings = int(min_swings)
        self.min_amplitude = float(min_amplitude)
        self.latch = float(latch)
        self.max_gap = float(max_gap)
        self.history = []          # [(t, x)]
        self.last_open_t = None
        self.latched_until = -1.0
        self.last_swings = 0

    def reset(self):
        self.history = []
        self.last_open_t = None
        self.latched_until = -1.0
        self.last_swings = 0

    def update(self, t, x, is_open):
        """t: thời gian (s); x: hoành độ tâm bàn tay (0..1) hoặc None nếu không thấy tay;
        is_open: bàn tay đang xoè. Trả True nếu đang ở trạng thái "đang vẫy"."""
        open_now = x is not None and bool(is_open)
        if self.last_open_t is not None and t - self.last_open_t > self.max_gap:
            # mất tay / tay không xoè quá lâu -> lần giơ tay sau là cử chỉ MỚI (không kế thừa "Wave")
            self.history = []
            self.latched_until = -1.0
        if open_now:
            self.history.append((t, float(x)))
            self.last_open_t = t
        self.history = [(ht, hx) for ht, hx in self.history if t - ht <= self.window]
        self.last_swings = count_swings([hx for _, hx in self.history], self.min_amplitude)
        if open_now and self.last_swings >= self.min_swings:
            self.latched_until = t + self.latch
        # chỉ báo "Wave" khi khung hiện tại vẫn là bàn tay xoè (ngón cái / nắm tay ngay sau khi vẫy
        # không bị gán nhầm thành Wave)
        return open_now and t < self.latched_until

    def confidence(self):
        """Độ tin cậy giả định của "Wave" để hiển thị: số nhịp / số nhịp yêu cầu (tối đa 1)."""
        if self.min_swings <= 0:
            return 1.0
        return min(1.0, max(self.last_swings, 0) / float(self.min_swings))


def frame_label(category, score, min_score, hand_present, is_waving):
    """Nhãn của MỘT khung hình (trước khi lọc ổn định).

    - đang vẫy -> 'Wave' (ưu tiên hơn Open_Palm cùng khung);
    - không có tay hoặc độ tin cậy < min_score -> 'None'."""
    if is_waving and hand_present:
        return WAVE
    if not hand_present:
        return NONE
    category = canonical_gesture(category)
    if category == NONE or score is None or score < min_score:
        return NONE
    return category


class GestureStabilizer:
    """Bộ lọc chống nhiễu theo thời gian (bỏ phiếu trong cửa sổ trượt).

    Một nhãn L chỉ trở thành kết quả "ổn định" khi trong `hold(L)` giây gần nhất:
      * đã quan sát đủ lâu (khung cũ nhất trong cửa sổ cách hiện tại >= 80 % hold),
      * có >= min_frames khung mang nhãn L,
      * tỉ lệ khung mang nhãn L >= vote_ratio.
    Các khung lẻ bị nhận nhầm (1-2 khung) không đủ phiếu -> không đổi kết quả.
    Open_Palm (dừng) có hold ngắn hơn để robot dừng nhanh; 'None' có hold riêng (mất tay).
    Khi nhiều nhãn cùng đạt, ưu tiên theo `priority` (dừng/huỷ trước).
    """

    def __init__(self, hold_time=0.5, hold_overrides=None, vote_ratio=0.7, min_frames=2,
                 priority=('Open_Palm', 'Closed_Fist')):
        self.hold_time = float(hold_time)
        self.hold_overrides = dict(hold_overrides or {})
        self.vote_ratio = float(vote_ratio)
        self.min_frames = max(1, int(min_frames))
        self.priority = tuple(priority)
        self.history = []          # [(t, label)]
        self.stable = NONE
        self.stable_since = None
        self.stable_ratio = 0.0

    def hold(self, label):
        return float(self.hold_overrides.get(label, self.hold_time))

    def reset(self):
        self.history = []
        self.stable = NONE
        self.stable_since = None
        self.stable_ratio = 0.0

    def _qualifies(self, label, t):
        h = self.hold(label)
        frames = [lb for ft, lb in self.history if t - ft <= h + 1e-9]
        if not frames:
            return None
        oldest = next(ft for ft, _ in self.history if t - ft <= h + 1e-9)
        if t - oldest < 0.8 * h - 1e-9:
            return None
        n = sum(1 for lb in frames if lb == label)
        ratio = n / float(len(frames))
        if n >= self.min_frames and ratio >= self.vote_ratio:
            return ratio
        return None

    def update(self, t, label):
        """Thêm nhãn của khung tại thời điểm t. Trả (nhãn_ổn_định, có_thay_đổi)."""
        label = canonical_gesture(label)
        self.history.append((t, label))
        horizon = max([self.hold_time] + list(self.hold_overrides.values())) + 0.5
        self.history = [(ft, lb) for ft, lb in self.history if t - ft <= horizon]
        candidates = {}
        for lb in set(lb for _, lb in self.history):
            r = self._qualifies(lb, t)
            if r is not None:
                candidates[lb] = r
        if not candidates:
            return self.stable, False
        ranked = sorted(candidates, key=lambda lb: (
            self.priority.index(lb) if lb in self.priority else len(self.priority), -candidates[lb]))
        winner = ranked[0]
        if self.stable in candidates and self.stable not in self.priority and winner not in self.priority:
            winner = self.stable                      # giữ nhãn hiện tại nếu nó vẫn đạt (tránh lật qua lại)
        changed = winner != self.stable
        if changed:
            self.stable = winner
            self.stable_since = t
        self.stable_ratio = candidates.get(self.stable, 0.0)
        return self.stable, changed


# ----------------------------------------------------------------------------------------------------
# Kế hoạch chuyển động (dùng cho robot_behavior_node)
# ----------------------------------------------------------------------------------------------------
class Segment:
    """Một đoạn chuyển động: vận tốc (lin m/s, ang rad/s) cho tới khi đạt `target`.

    kind = 'dist'  : target = quãng đường (m)   – đo bằng /odom
           'angle' : target = góc quay (rad)    – đo bằng /odom (tích luỹ yaw)
           'time'  : target = thời gian (s)     – dừng chờ (lin = ang = 0)
    Không có /odom -> 'dist'/'angle' quy ra thời gian = target / |vận tốc|."""

    __slots__ = ('name', 'lin', 'ang', 'target', 'kind')

    def __init__(self, name, lin, ang, target, kind):
        if kind not in ('dist', 'angle', 'time'):
            raise ValueError('kind không hợp lệ: {}'.format(kind))
        self.name, self.lin, self.ang, self.target, self.kind = name, float(lin), float(ang), \
            float(target), kind

    def nominal_duration(self):
        if self.kind == 'time':
            return self.target
        speed = abs(self.lin) if self.kind == 'dist' else abs(self.ang)
        return self.target / speed if speed > 1e-6 else 0.0

    def __repr__(self):
        return 'Segment({}, lin={:.2f}, ang={:.2f}, {}={:.2f})'.format(
            self.name, self.lin, self.ang, self.kind, self.target)


DEFAULT_BEHAVIOR_PARAMS = {
    'greet_angle': 0.6,          # rad (~34°) mỗi bên
    'greet_speed': 1.0,          # rad/s
    'greet_repeats': 2.0,        # số lần trái-phải
    'fb_distance': 0.35,         # m mỗi lần tiến (và lùi)
    'fb_speed': 0.2,             # m/s
    'fb_repeats': 4.0,           # "tiến-lùi 4 lần"
    'circle_radius': 0.5,        # m
    'circle_speed': 0.15,        # m/s
    'circle_laps': 1.0,          # số vòng
    'pause': 0.3,                # s nghỉ giữa các đoạn
}


def build_plan(behavior, params=None):
    """Danh sách Segment cho một hành vi. Hành vi đối xứng -> robot kết thúc gần chỗ bắt đầu."""
    p = dict(DEFAULT_BEHAVIOR_PARAMS)
    p.update(params or {})
    pause = Segment('nghi', 0.0, 0.0, p['pause'], 'time')
    plan = []
    if behavior == 'greet':
        a, w = abs(p['greet_angle']), abs(p['greet_speed'])
        plan.append(Segment('chao: quay trai', 0.0, w, a, 'angle'))
        for i in range(int(p['greet_repeats'])):
            plan.append(Segment('chao: quay phai', 0.0, -w, 2 * a, 'angle'))
            last = i == int(p['greet_repeats']) - 1
            plan.append(Segment('chao: quay trai', 0.0, w, a if last else 2 * a, 'angle'))
    elif behavior == 'forward_back':
        d, v = abs(p['fb_distance']), abs(p['fb_speed'])
        n = int(p['fb_repeats'])
        for i in range(n):
            plan.append(Segment('tien {}/{}'.format(i + 1, n), v, 0.0, d, 'dist'))
            plan.append(pause)
            plan.append(Segment('lui {}/{}'.format(i + 1, n), -v, 0.0, d, 'dist'))
            if i < n - 1:
                plan.append(pause)
    elif behavior == 'circle':
        r, v = abs(p['circle_radius']), abs(p['circle_speed'])
        if r < 0.05:
            raise ValueError('circle_radius quá nhỏ')
        plan.append(Segment('quy dao tron r={:.2f} m'.format(r), v, v / r,
                            2.0 * math.pi * float(p['circle_laps']), 'angle'))
    else:
        raise ValueError('hành vi không tồn tại: {}'.format(behavior))
    return plan


def plan_extent(plan):
    """Mô phỏng động học (không quán tính) -> (bán kính lớn nhất so với điểm xuất phát, pose cuối).

    Dùng để kiểm tra trước hành vi nằm gọn trong vùng an toàn (unit test + log khi khởi động)."""
    x = y = th = 0.0
    rmax = 0.0
    dt = 0.01
    for seg in plan:
        steps = int(round(seg.nominal_duration() / dt))
        for _ in range(steps):
            x += seg.lin * math.cos(th) * dt
            y += seg.lin * math.sin(th) * dt
            th += seg.ang * dt
            rmax = max(rmax, math.hypot(x, y))
    return rmax, (x, y, math.atan2(math.sin(th), math.cos(th)))


def clear_dock_plan(pose, radius, speed=0.2, turn_speed=0.8):
    """Kế hoạch đưa robot ra xa dock khi bước chuẩn bị bị huỷ giữa chừng.

    Robot khởi động trong dock tại gốc odom, nên dock ở quanh (0, 0). Quay tại chỗ cho mặt hướng RA XA gốc
    (theo tia từ gốc qua vị trí robot; robot ở sát gốc -> hướng -x như sau khi undock), rồi chạy thẳng tới khi
    cách gốc `radius` m. pose = (x, y, yaw) trong frame odom."""
    x, y, yaw = pose
    r = math.hypot(x, y)
    heading = math.atan2(y, x) if r > 0.05 else math.pi
    dyaw = angle_diff(heading, yaw)
    plan = []
    if abs(dyaw) > 0.05:
        plan.append(Segment('quay ra xa dock', 0.0, math.copysign(abs(turn_speed), dyaw), abs(dyaw), 'angle'))
    if radius - r > 0.02:
        plan.append(Segment('ra giua san', abs(speed), 0.0, radius - r, 'dist'))
    if not plan:
        plan.append(Segment('nghi', 0.0, 0.0, 0.05, 'time'))
    return plan


def angle_diff(a, b):
    """a - b đưa về (-pi, pi]."""
    d = math.fmod(a - b + math.pi, 2.0 * math.pi)
    if d <= 0.0:
        d += 2.0 * math.pi
    return d - math.pi


class SegmentRunner:
    """Thực hiện tuần tự các Segment, tính lệnh vận tốc mỗi chu kỳ điều khiển (thuần Python).

    - đo tiến độ bằng pose /odom (x, y, yaw); không có odom -> theo thời gian;
    - tăng tốc dần (max_lin_acc, max_ang_acc) và giảm tốc gần đích (v <= sqrt(2·a·quãng còn lại))
      để không vọt quá đích;
    - kẹt (lệnh chạy mà pose không đổi trong stall_timeout) hoặc quá thời gian -> báo lỗi.
    """

    def __init__(self, plan, max_lin_acc=0.5, max_ang_acc=2.0, min_lin=0.03, min_ang=0.15,
                 stall_timeout=4.0, timeout_factor=4.0, timeout_margin=5.0, lag=0.15, period=0.05):
        self.plan = list(plan)
        self.lag = float(lag)          # s – trễ đáp ứng của robot (bù khi giảm tốc để không vọt đích)
        self.period = float(period)    # s – chu kỳ điều khiển dự kiến (dùng cho lần gọi đầu tiên)
        self.max_lin_acc = float(max_lin_acc)
        self.max_ang_acc = float(max_ang_acc)
        self.min_lin = float(min_lin)
        self.min_ang = float(min_ang)
        self.stall_timeout = float(stall_timeout)
        self.timeout_factor = float(timeout_factor)
        self.timeout_margin = float(timeout_margin)
        self.index = -1
        self.done = False
        self.error = None
        self.cmd = (0.0, 0.0)
        self.used_odom = None
        self._seg_t0 = None
        self._seg_pose0 = None
        self._last_pose = None
        self._yaw_acc = 0.0
        self._progress = 0.0
        self._last_move_t = None
        self._last_t = None
        self.total_nominal = sum(s.nominal_duration() for s in self.plan)

    # ------------------------------------------------------------------
    def current(self):
        return self.plan[self.index] if 0 <= self.index < len(self.plan) else None

    def _start_segment(self, t, pose):
        self.index += 1
        if self.index >= len(self.plan):
            self.done = True
            self.cmd = (0.0, 0.0)
            return
        self._seg_t0 = t
        self._seg_pose0 = pose
        self._last_pose = pose
        self._yaw_acc = 0.0
        self._progress = 0.0
        self._last_move_t = t

    def overall_progress(self):
        if self.done:
            return 1.0
        if self.total_nominal <= 0 or self.index < 0:
            return 0.0
        before = sum(s.nominal_duration() for s in self.plan[:self.index])
        seg = self.current()
        frac = 0.0
        if seg is not None and seg.target > 0:
            frac = min(1.0, self._progress / seg.target)
        return min(1.0, (before + frac * seg.nominal_duration()) / self.total_nominal)

    def update(self, t, pose):
        """t: thời gian (s); pose: (x, y, yaw) mới (≤ ~0.5 s) hoặc None. Trả (lin, ang)."""
        if self.done or self.error:
            self.cmd = (0.0, 0.0)
            return self.cmd
        if self.index < 0:
            self._start_segment(t, pose)
            if self.done:
                return self.cmd
        dt = self.period if self._last_t is None else max(0.0, t - self._last_t)
        self._last_t = t
        # Tối đa vài đoạn mỗi chu kỳ (đoạn có target 0 kết thúc ngay).
        for _ in range(len(self.plan) + 1):
            seg = self.current()
            if seg is None:
                self.done = True
                self.cmd = (0.0, 0.0)
                return self.cmd
            if self._seg_pose0 is None and pose is not None and seg.kind != 'time':
                # odom tới muộn: bắt đầu đo từ bây giờ
                self._seg_pose0 = pose
                self._last_pose = pose
            remaining = self._measure(t, pose, seg)
            if remaining <= 1e-3:
                self._start_segment(t, pose)
                if self.done:
                    return self.cmd
                continue
            break
        seg = self.current()
        elapsed = t - self._seg_t0
        limit = self.timeout_factor * seg.nominal_duration() + self.timeout_margin
        if elapsed > limit:
            self.error = 'quá thời gian ở đoạn "{}" ({:.1f} s)'.format(seg.name, elapsed)
            self.cmd = (0.0, 0.0)
            return self.cmd
        if seg.kind != 'time' and self.used_odom and t - self._last_move_t > self.stall_timeout:
            self.error = 'robot không nhúc nhích {:.1f} s ở đoạn "{}" (kẹt / va chạm / mất /odom?)'.format(
                t - self._last_move_t, seg.name)
            self.cmd = (0.0, 0.0)
            return self.cmd
        self.cmd = self._command(seg, remaining, dt)
        return self.cmd

    def _measure(self, t, pose, seg):
        """Cập nhật tiến độ của đoạn hiện tại; trả phần còn lại (đơn vị của target)."""
        if seg.kind == 'time':
            self._progress = t - self._seg_t0
            return seg.target - self._progress
        if pose is not None and self._seg_pose0 is not None:
            self.used_odom = True
            if seg.kind == 'dist':
                prog = math.hypot(pose[0] - self._seg_pose0[0], pose[1] - self._seg_pose0[1])
                moved = math.hypot(pose[0] - self._last_pose[0], pose[1] - self._last_pose[1])
                if moved > 0.003:
                    self._last_move_t = t
                    self._last_pose = pose
            else:
                d = angle_diff(pose[2], self._last_pose[2])
                if abs(d) > 0.003:
                    self._yaw_acc += abs(d)
                    self._last_move_t = t
                    self._last_pose = pose
                prog = self._yaw_acc
                # bám thêm dịch chuyển (đoạn cung tròn) để phát hiện kẹt
                if seg.lin != 0.0 and math.hypot(pose[0] - self._last_pose[0],
                                                 pose[1] - self._last_pose[1]) > 0.003:
                    self._last_move_t = t
            self._progress = max(self._progress, prog)
        elif self.used_odom:
            # đã đo bằng odom mà odom tạm mất: GIỮ tiến độ (không nhảy sang ước lượng theo thời gian, dễ kết thúc
            # đoạn sớm). Mất quá stall_timeout -> báo lỗi ở update().
            pass
        else:
            self.used_odom = False
            speed = abs(seg.lin) if seg.kind == 'dist' else abs(seg.ang)
            self._progress = (t - self._seg_t0) * speed
        return seg.target - self._progress

    def _command(self, seg, remaining, dt):
        if seg.kind == 'time':
            return (0.0, 0.0)
        lin, ang = seg.lin, seg.ang
        plin, pang = self.cmd
        # giảm tốc gần đích theo đại lượng điều khiển chính: v <= sqrt(2·a·s), với s = quãng còn lại
        # trừ phần robot còn trôi do trễ (|v trước|·lag)
        if seg.kind == 'dist' and abs(lin) > 0:
            rem = max(0.0, remaining - abs(plin) * self.lag)
            vmax = max(self.min_lin, math.sqrt(2.0 * self.max_lin_acc * rem))
            scale = min(1.0, vmax / abs(lin))
        elif seg.kind == 'angle' and abs(ang) > 0:
            rem = max(0.0, remaining - abs(pang) * self.lag)
            wmax = max(self.min_ang, math.sqrt(2.0 * self.max_ang_acc * rem))
            scale = min(1.0, wmax / abs(ang))
        else:
            scale = 1.0
        lin, ang = lin * scale, ang * scale
        # giới hạn gia tốc so với lệnh trước (đổi chiều cũng đi qua 0 từ từ)
        if dt > 0:
            lin = plin + max(-self.max_lin_acc * dt, min(self.max_lin_acc * dt, lin - plin))
            ang = pang + max(-self.max_ang_acc * dt, min(self.max_ang_acc * dt, ang - pang))
        return (lin, ang)


# ----------------------------------------------------------------------------------------------------
# Máy trạng thái hành vi
# ----------------------------------------------------------------------------------------------------
WAITING = 'WAITING'          # chưa nhận tín hiệu cử chỉ nào
SETUP = 'SETUP'              # đang rời dock / ra giữa sân
IDLE = 'IDLE'                # chờ lệnh, chưa chọn hành vi
SELECTED = 'SELECTED'        # đã chọn, chờ ngón cái xác nhận
EXECUTING = 'EXECUTING'      # đang thực hiện
STOPPED = 'STOPPED'          # vừa dừng khẩn (xoè tay) – vẫn nhớ hành vi đã chọn
SIGNAL_LOST = 'SIGNAL_LOST'  # mất tín hiệu điều khiển


class BehaviorStateMachine:
    """Quyết định robot làm gì khi nhận cử chỉ (thuần Python – node ROS chỉ thực thi các action trả về).

    Action trả về (list tuple):
      ('stop', lý_do)          dừng NGAY (gửi Twist 0), huỷ hành vi đang chạy nếu có
      ('start', hành_vi)       bắt đầu thực hiện hành vi
      ('abort_setup', lý_do)   huỷ bước chuẩn bị (undock / ra giữa sân)
      ('info', thông_điệp)     chỉ ghi log

    Chống kích hoạt liên tục:
      * chỉ phản ứng khi nhãn ỔN ĐỊNH đổi sang cử chỉ mới (sườn lên) – giữ nguyên ngón cái 10 s cũng
        chỉ tính 1 lần; muốn ra lệnh lại phải đổi cử chỉ / hạ tay;
      * cùng một cử chỉ lệnh không có hiệu lực lại trong `cooldown` giây (trừ xoè tay = dừng);
      * hành vi chỉ chạy sau ngón cái xác nhận; lựa chọn hết hạn sau `selection_timeout` giây;
      * đang chạy thì cử chỉ chọn/xác nhận bị bỏ qua (chỉ dừng hoặc huỷ mới có tác dụng).
    """

    def __init__(self, wave_requires_confirm=True, selection_timeout=15.0, cooldown=1.0,
                 signal_timeout=1.0):
        self.wave_requires_confirm = bool(wave_requires_confirm)
        self.selection_timeout = float(selection_timeout)
        self.cooldown = float(cooldown)
        self.signal_timeout = float(signal_timeout)
        self.state = WAITING
        self.selected = None
        self.active = None
        self.selected_t = None
        self.last_label = None
        self.last_msg_t = None
        self.last_trigger = {}
        self.message = 'Cho tin hieu cu chi tu /detected_gesture'
        self.setup_pending_state = IDLE
        self.counts = {}

    # ------------------------------------------------------------------ tiện ích
    def _set(self, state, message=None):
        self.state = state
        if message is not None:
            self.message = message

    def describe(self):
        return {'state': self.state, 'selected': self.selected, 'active': self.active,
                'message': self.message}

    # ------------------------------------------------------------------ sự kiện
    def begin_setup(self, message='Dang roi dock / ra giua san'):
        self.setup_pending_state = IDLE if self.state != WAITING else WAITING
        self._set(SETUP, message)

    def end_setup(self, ok=True, message=None):
        if self.state != SETUP:
            return
        nxt = IDLE if self.last_msg_t is not None else WAITING
        self._set(nxt, message or ('San sang nhan cu chi' if ok else 'Bo qua buoc chuan bi'))

    def on_message(self, label, t):
        """Mọi tin nhắn trên /detected_gesture (kể cả lặp lại) – dùng cho watchdog + phát hiện sườn."""
        label = canonical_gesture(label)
        actions = []
        first = self.last_msg_t is None
        self.last_msg_t = t
        if self.state == WAITING:
            self._set(IDLE, 'Da nhan tin hieu cu chi - san sang')
        elif self.state == SIGNAL_LOST:
            self._set(IDLE, 'Tin hieu cu chi da tro lai - san sang')
            self.last_label = None
            actions.append(('info', 'Tín hiệu cử chỉ đã trở lại -> IDLE'))
        if first:
            self.last_label = None
        if label != self.last_label:
            self.last_label = label
            actions += self._on_gesture(label, t)
        return actions

    def _cooling(self, label, t):
        last = self.last_trigger.get(label)
        return last is not None and t - last < self.cooldown

    def _on_gesture(self, g, t):
        if g == NONE:
            return []
        if g == STOP_GESTURE:                         # luôn có hiệu lực, không cooldown
            self.last_trigger[g] = t
            self.counts[g] = self.counts.get(g, 0) + 1
            if self.state == SETUP:
                return [('stop', 'xoe tay'), ('abort_setup', 'xoè tay khi đang chuẩn bị')]
            if self.state == EXECUTING:
                self.active = None
                self.selected_t = t           # tính lại hạn lựa chọn từ lúc dừng (hành vi có thể đã chạy lâu)
                self._set(STOPPED, 'DUNG KHAN (xoe tay). Ngon cai = chay lai, nam tay = huy')
                return [('stop', 'xoe tay (dung khan)')]
            msg = 'Robot dang dung (xoe tay)'
            if self.state == SELECTED:
                msg = 'Dung. Van giu lua chon: {}'.format(self.selected)
            self.message = msg
            return [('stop', 'xoe tay')]
        if self._cooling(g, t):
            return [('info', 'Bỏ qua {} (cooldown {:.1f} s)'.format(g, self.cooldown))]
        self.last_trigger[g] = t
        self.counts[g] = self.counts.get(g, 0) + 1
        if g == CANCEL_GESTURE:
            if self.state == SETUP:
                return [('stop', 'nam tay'), ('abort_setup', 'nắm tay khi đang chuẩn bị')]
            was = self.active or self.selected
            self.active = None
            self.selected = None
            self.selected_t = None
            self._set(IDLE, 'Da huy{} - ve trang thai cho'.format(' ' + was if was else ''))
            return [('stop', 'nam tay (huy)')]
        if self.state == SETUP:
            return [('info', 'Đang chuẩn bị – bỏ qua {}'.format(g))]
        if self.state == EXECUTING:
            return [('info', 'Đang thực hiện {} – bỏ qua {} (xoè tay để dừng, nắm tay để huỷ)'.format(
                self.active, g))]
        if g in SELECT_GESTURES:
            behavior = SELECT_GESTURES[g]
            if g == WAVE and not self.wave_requires_confirm:
                self.selected = None
                self.active = behavior
                self._set(EXECUTING, 'Chao lai!')
                return [('start', behavior)]
            self.selected = behavior
            self.selected_t = t
            self._set(SELECTED, 'Da chon: {} -> gio NGON CAI de xac nhan'.format(behavior))
            return [('info', 'Đã chọn hành vi {} (chờ ngón cái xác nhận)'.format(behavior))]
        if g == CONFIRM_GESTURE:
            if self.state in (SELECTED, STOPPED) and self.selected:
                self.active = self.selected
                self._set(EXECUTING, 'Dang thuc hien: {}'.format(self.active))
                return [('start', self.active)]
            self.message = 'Chua chon hanh vi - gio 1 ngon / 2 ngon / vay tay truoc'
            return [('info', 'Ngón cái nhưng chưa chọn hành vi – bỏ qua')]
        return [('info', 'Cử chỉ {} không gắn với hành vi nào'.format(g))]

    def on_behavior_finished(self, ok=True, reason=''):
        """Node báo hành vi đã xong (ok) hoặc bị lỗi (kẹt, vật cản, quá thời gian)."""
        if self.state != EXECUTING:
            return []
        name = self.active
        self.active = None
        self.selected = None          # muốn chạy lại phải chọn + xác nhận lại (an toàn hơn)
        self.selected_t = None
        if ok:
            self._set(IDLE, 'Xong: {} - cho lenh moi'.format(name))
            return [('info', 'Hoàn thành {}'.format(name))]
        self._set(STOPPED, 'Dung: {} ({})'.format(name, reason))
        return [('stop', reason or 'lỗi khi thực hiện')]

    def tick(self, t):
        """Gọi định kỳ: watchdog mất tín hiệu + hết hạn lựa chọn."""
        actions = []
        if (self.last_msg_t is not None and self.state not in (WAITING, SIGNAL_LOST)
                and t - self.last_msg_t > self.signal_timeout):
            was_moving = self.state in (EXECUTING, SETUP)
            if self.state == SETUP:
                actions.append(('abort_setup', 'mất tín hiệu cử chỉ'))
            self.active = None
            self.selected = None
            self.selected_t = None
            self._set(SIGNAL_LOST, 'MAT TIN HIEU cu chi > {:.1f} s - robot dung'.format(self.signal_timeout))
            actions.append(('stop', 'mất tín hiệu điều khiển ({:.1f} s không có /detected_gesture){}'.format(
                t - self.last_msg_t, ' – huỷ hành vi đang chạy' if was_moving else '')))
            return actions
        if (self.state in (SELECTED, STOPPED) and self.selected and self.selected_t is not None
                and t - self.selected_t > self.selection_timeout):
            name = self.selected
            self.selected = None
            self.selected_t = None
            self._set(IDLE, 'Lua chon {} da het han ({:.0f} s)'.format(name, self.selection_timeout))
            actions.append(('info', 'Lựa chọn {} hết hạn sau {:.0f} s không xác nhận'.format(
                name, self.selection_timeout)))
        return actions


def obstacle_in_path(ranges, angle_min, angle_increment, range_min, range_max, lidar_yaw,
                     direction, stop_distance, half_angle):
    """Có vật cản gần hơn stop_distance trong hình quạt ±half_angle quanh hướng chuyển động?

    direction: +1 tiến, -1 lùi. lidar_yaw: góc quay của frame LiDAR so với base_link (TurtleBot 4:
    +pi/2). Trả (True/False, khoảng cách gần nhất trong quạt hoặc None)."""
    if not ranges or direction == 0:
        return False, None
    heading = 0.0 if direction > 0 else math.pi
    nearest = None
    for i, r in enumerate(ranges):
        if r is None or not math.isfinite(r) or r < max(range_min, 0.05) or r > range_max:
            continue
        a = angle_min + i * angle_increment + lidar_yaw
        if abs(angle_diff(a, heading)) <= half_angle:
            if nearest is None or r < nearest:
                nearest = r
    return (nearest is not None and nearest < stop_distance), nearest
