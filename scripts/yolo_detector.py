#!/usr/bin/env python3
"""Phần 4 – Nhận diện vật thể từ camera RGB của TurtleBot 4 bằng YOLO (ultralytics).

  ros2 run tb4_lab04 yolo_detector.py
  ros2 launch tb4_lab04 yolo.launch.py                   # đọc tham số từ config/yolo.yaml
  ros2 run tb4_lab04 yolo_detector.py --ros-args -p model:=yolov8n.pt -p conf:=0.3

Luồng xử lý:
  /oakd/rgb/preview/image_raw (sensor_msgs/msg/Image, rgb8 320x240)
    -> cv_bridge.imgmsg_to_cv2(msg, 'bgr8')                  (ảnh OpenCV)
    -> YOLO (mặc định yolo11n.pt – 80 lớp COCO; hoặc YOLO-World với danh sách lớp tự đặt)
    -> vẽ bounding box + tên lớp + độ tự tin
    -> cửa sổ OpenCV  +  /yolo/image_annotated (Image, xem trên RViz2)
       +  /yolo/detections (vision_msgs/msg/Detection2DArray)
    -> in thông tin vật thể ra terminal (kèm vị trí robot trên bản đồ từ /amcl_pose)
    -> lưu ~/tb4_results/yolo_<thời_gian>/: detections.csv, first_<lớp>.jpg, best_<lớp>.jpg
Suy luận chạy ở luồng riêng, chỉ lấy khung hình MỚI NHẤT (bỏ khung cũ) -> không bị trễ dồn khi CPU yếu.

Tham số chính (xem config/yolo.yaml): image_topic, model, conf, iou, imgsz, device, max_rate,
classes, world_classes, show_window, window_scale, print_period, cpu_threads, save_results.
"""

import csv
import os
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
import lab_common as lc  # noqa: E402

WINDOW = 'YOLO - camera TurtleBot 4'
DEFAULT_WORLD_CLASSES = ['person', 'car', 'chair', 'fire hydrant', 'fire extinguisher']
NUMPY2_HINT = ('cv_bridge của ROS 2 Humble (apt) được build với NumPy 1.x. Nếu pip đã cài NumPy 2.x '
               '(do `pip install ultralytics opencv-python` kéo về), cv_bridge sẽ lỗi. Sửa:\n'
               '    pip3 install --user "numpy<2" "opencv-python<4.11"\n'
               'Kiểm tra: ros2 run tb4_lab04 doctor.sh')


# ------------------------------------------------------------------ logic thuần (có unit test)
def class_color(name):
    """Màu BGR cố định cho mỗi lớp (cùng lớp -> cùng màu giữa các khung hình)."""
    h = 0
    for ch in name:
        h = (h * 131 + ord(ch)) & 0xFFFFFF
    palette = [(56, 56, 255), (151, 157, 255), (31, 112, 255), (29, 178, 255), (49, 210, 207),
               (10, 249, 72), (23, 204, 146), (134, 219, 61), (52, 147, 26), (187, 212, 0),
               (168, 153, 44), (255, 194, 0), (147, 69, 52), (255, 115, 100), (236, 24, 0),
               (255, 56, 132), (133, 0, 82), (255, 56, 203), (200, 149, 255), (199, 55, 255)]
    return palette[h % len(palette)]


def resolve_model_path(model, weights_dir):
    """Tên trần (yolo11n.pt) -> <weights_dir>/yolo11n.pt (ultralytics tự tải về đúng chỗ đó lần đầu);
    đường dẫn có thư mục hoặc file đã tồn tại -> giữ nguyên."""
    model = os.path.expanduser(model.strip())
    if os.path.isabs(model) or os.path.dirname(model) or os.path.exists(model):
        return model
    return os.path.join(os.path.expanduser(weights_dir), model)


def split_list(value):
    """'person, car' hoặc ['person', 'car'] -> ['person', 'car'] (bỏ phần tử rỗng)."""
    if isinstance(value, (list, tuple)):
        items = value
    else:
        items = str(value or '').split(',')
    return [str(v).strip() for v in items if str(v).strip()]


def is_world_model(path):
    return 'world' in os.path.basename(path).lower()


class DetectionLog:
    """Thống kê theo lớp: số khung có lớp đó, độ tự tin cao nhất, lần đầu thấy (thuần Python)."""

    def __init__(self):
        self.stats = {}

    def add(self, cls, conf, t, robot):
        s = self.stats.get(cls)
        new = s is None
        if new:
            s = {'frames': 0, 'max_conf': 0.0, 'first_t': t, 'first_robot': robot, 'best_new': False}
            self.stats[cls] = s
        s['frames'] += 1
        s['best_new'] = conf > s['max_conf']
        if s['best_new']:
            s['max_conf'] = conf
        return new, s['best_new']

    def lines(self):
        out = ['{:<18} {:>8} {:>9} {:>10}  {}'.format('lớp', 'số khung', 'conf max', 'lần đầu (s)',
                                                      'robot lúc thấy lần đầu (map)')]
        for cls in sorted(self.stats, key=lambda c: self.stats[c]['first_t']):
            s = self.stats[cls]
            rb = s['first_robot']
            rbs = '-' if rb is None else '({:.2f}, {:.2f})'.format(rb[0], rb[1])
            out.append('{:<18} {:>8} {:>9.2f} {:>10.1f}  {}'.format(cls, s['frames'], s['max_conf'],
                                                                   s['first_t'], rbs))
        return out


def format_detections(dets):
    return ', '.join('{} {:.2f} [{:.0f},{:.0f},{:.0f},{:.0f}]'.format(d[0], d[1], *d[2]) for d in dets)


# ------------------------------------------------------------------ vẽ
def draw(img, dets, hud_lines, cv2):
    """Vẽ bbox + nhãn lên ảnh BGR (tại chỗ)."""
    h, w = img.shape[:2]
    thick = max(1, int(round(min(h, w) / 240)))
    font_scale = 0.45 * max(1.0, min(h, w) / 300.0)
    for name, conf, (x1, y1, x2, y2) in dets:
        color = class_color(name)
        p1, p2 = (int(x1), int(y1)), (int(x2), int(y2))
        cv2.rectangle(img, p1, p2, color, thick + 1)
        label = '{} {:.2f}'.format(name, conf)
        (tw, th), base = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, font_scale, thick)
        ty = p1[1] - 4 if p1[1] - th - 6 > 0 else p1[1] + th + 6
        cv2.rectangle(img, (p1[0], ty - th - 4), (p1[0] + tw + 4, ty + base - 2), color, -1)
        cv2.putText(img, label, (p1[0] + 2, ty - 2), cv2.FONT_HERSHEY_SIMPLEX, font_scale,
                    (255, 255, 255), thick, cv2.LINE_AA)
    # HUD ở góc DƯỚI-trái: nhãn của bbox chạm mép trên ảnh (người đứng gần) không bị che
    (_, th0), _ = cv2.getTextSize('Ag', cv2.FONT_HERSHEY_SIMPLEX, font_scale, thick)
    y = h - 8 - (th0 + 10) * (len(hud_lines) - 1)
    for line in hud_lines:
        (tw, th), _ = cv2.getTextSize(line, cv2.FONT_HERSHEY_SIMPLEX, font_scale, thick)
        cv2.rectangle(img, (4, y - th - 4), (8 + tw, y + 4), (0, 0, 0), -1)
        cv2.putText(img, line, (6, y), cv2.FONT_HERSHEY_SIMPLEX, font_scale, (255, 255, 255), thick,
                    cv2.LINE_AA)
        y += th + 10
    return img


# ------------------------------------------------------------------ node
def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from sensor_msgs.msg import Image
    import numpy as np
    if int(np.__version__.split('.')[0]) >= 2:
        # Phải chặn TRƯỚC khi import cv_bridge: với NumPy 2.x, module C++ của cv_bridge (build với
        # NumPy 1.x) làm tiến trình segfault ở ảnh đầu tiên – không kịp in lỗi (đã kiểm chứng).
        print('[yolo_detector] NumPy {} không dùng được với cv_bridge của ROS 2 Humble.\n{}'.format(
            np.__version__, NUMPY2_HINT), flush=True)
        return 2
    import cv2
    try:
        from cv_bridge import CvBridge
    except Exception as exc:  # noqa: BLE001 (ImportError hoặc lỗi ABI NumPy)
        print('[yolo_detector] KHÔNG import được cv_bridge: {}\n{}'.format(exc, NUMPY2_HINT), flush=True)
        return 2
    try:
        from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
    except ImportError:
        Detection2D = Detection2DArray = ObjectHypothesisWithPose = None

    # Tự kiểm tra cv_bridge NGAY: với NumPy 2.x, `import cv_bridge` vẫn chạy nhưng imgmsg_to_cv2 lỗi
    # "_ARRAY_API not found" ở ảnh đầu tiên (đã kiểm chứng: numpy 2.2.6 + cv_bridge Humble).
    try:
        probe = Image(height=2, width=2, encoding='rgb8', step=6, data=bytes(12))
        CvBridge().imgmsg_to_cv2(probe, desired_encoding='bgr8')
    except Exception as exc:  # noqa: BLE001
        print('[yolo_detector] cv_bridge KHÔNG chuyển được ảnh (NumPy {}): {}: {}\n{}'.format(
            np.__version__, type(exc).__name__, exc, NUMPY2_HINT), flush=True)
        return 2

    # Tự xử lý SIGINT/SIGTERM bằng CỜ DỪNG thay vì để rclpy đóng context + ném KeyboardInterrupt: ngoại lệ
    # rơi vào giữa khối finally (đang chờ luồng YOLO) làm mất bảng tổng kết, và Python thoát khi luồng
    # PyTorch còn chạy -> "terminate called without an active exception" (exit -6). Đã tái hiện qua ros2 launch.
    import signal
    stop_flag = {'stop': False}

    def _request_stop(signum, frame):
        if stop_flag['stop']:
            return
        stop_flag['stop'] = True
        lc.SIGINT_STATE['got'] = True
    try:
        from rclpy.signals import SignalHandlerOptions
        rclpy.init(args=sys.argv, signal_handler_options=SignalHandlerOptions.NO)
    except (ImportError, TypeError):
        rclpy.init(args=sys.argv)
    signal.signal(signal.SIGINT, _request_stop)
    signal.signal(signal.SIGTERM, _request_stop)
    node = Node('yolo_detector',
                parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])
    log = node.get_logger()
    P = node.declare_parameter
    P('image_topic', '/oakd/rgb/preview/image_raw')
    P('model', 'yolo11n.pt')
    P('weights_dir', '~/.cache/tb4_lab04/models')
    P('conf', 0.35)
    P('iou', 0.5)
    P('imgsz', 640)
    P('device', '')
    P('max_rate', 5.0)
    P('classes', '')            # chuỗi, phân tách bằng dấu phẩy: "person,car" ('' = mọi lớp)
    P('world_classes', '')      # chỉ cho YOLO-World: "person,car,fire extinguisher"
    P('show_window', True)
    P('window_scale', 2.0)
    P('publish_annotated', True)
    P('print_period', 1.0)
    P('cpu_threads', 0)
    P('save_results', True)
    P('results_dir', '')
    prm = {k: node.get_parameter(k).value for k in (
        'image_topic', 'model', 'weights_dir', 'conf', 'iou', 'imgsz', 'device', 'max_rate', 'classes',
        'world_classes', 'show_window', 'window_scale', 'publish_annotated', 'print_period',
        'cpu_threads', 'save_results', 'results_dir')}
    prm['classes'] = split_list(prm['classes'])
    prm['world_classes'] = split_list(prm['world_classes'])

    # ---------------- model
    try:
        import torch
        from ultralytics import YOLO
    except Exception as exc:  # noqa: BLE001
        log.error('Không import được ultralytics/torch: {}\n  Cài: pip3 install --user ultralytics '
                  '"numpy<2" "opencv-python<4.11"  (README mục cài đặt)'.format(exc))
        return 2
    threads = int(prm['cpu_threads']) or max(1, (os.cpu_count() or 2) // 2)
    torch.set_num_threads(threads)
    model_path = resolve_model_path(prm['model'], prm['weights_dir'])
    os.makedirs(os.path.dirname(model_path) or '.', exist_ok=True)
    log.info('Nạp model YOLO: {} (lần đầu sẽ tự tải về – cần Internet) ...'.format(model_path))
    t0 = time.monotonic()
    try:
        model = YOLO(model_path)
    except Exception as exc:  # noqa: BLE001
        log.error('Không nạp được model {}: {}'.format(model_path, exc))
        return 2
    if is_world_model(model_path):
        wc = prm['world_classes'] or DEFAULT_WORLD_CLASSES
        try:
            model.set_classes(wc)
        except Exception as exc:  # noqa: BLE001
            log.error('YOLO-World set_classes lỗi (cần gói CLIP, ultralytics tự cài khi có Internet): '
                      '{}'.format(exc))
            return 2
        log.info('YOLO-World – lớp tìm kiếm: {}'.format(', '.join(wc)))
    names = model.names if isinstance(model.names, dict) else dict(enumerate(model.names))
    class_ids = None
    if prm['classes']:
        lookup = {v: k for k, v in names.items()}
        unknown = [c for c in prm['classes'] if c not in lookup]
        if unknown:
            log.warn('Lớp không có trong model, bỏ qua: {}'.format(', '.join(unknown)))
        class_ids = [lookup[c] for c in prm['classes'] if c in lookup] or None
    device = prm['device'] or None
    try:
        model.predict(np.zeros((240, 320, 3), dtype=np.uint8), imgsz=int(prm['imgsz']), device=device,
                      verbose=False)                           # khởi động (lần đầu chậm)
    except Exception as exc:  # noqa: BLE001
        log.error('Chạy thử model lỗi: {}'.format(exc))
        return 2
    t1 = time.monotonic()
    model.predict(np.zeros((240, 320, 3), dtype=np.uint8), imgsz=int(prm['imgsz']), device=device,
                  verbose=False)
    log.info('Model sẵn sàng sau {:.1f} s: {} lớp, thiết bị {}, {} luồng CPU, ~{:.0f} ms/ảnh'.format(
        t1 - t0, len(names), device or ('cuda' if torch.cuda.is_available() else 'cpu'), threads,
        (time.monotonic() - t1) * 1000))
    model_label = os.path.splitext(os.path.basename(model_path))[0]

    # ---------------- ROS I/O
    bridge = CvBridge()
    lock = threading.Lock()
    state = {'msg': None, 'seq': 0, 'robot': None, 'stop': False, 'last_img_wall': None}

    def on_image(msg):
        with lock:
            state['msg'] = msg
            state['seq'] += 1
            state['last_img_wall'] = time.monotonic()

    def on_amcl(msg):
        q = msg.pose.pose.orientation
        state['robot'] = (msg.pose.pose.position.x, msg.pose.pose.position.y,
                          lc.yaw_from_quat(q.x, q.y, q.z, q.w))

    node.create_subscription(Image, prm['image_topic'], on_image, qos_profile_sensor_data)
    node.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', on_amcl,
                             QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                        reliability=ReliabilityPolicy.RELIABLE))
    pub_img = node.create_publisher(Image, '/yolo/image_annotated', 5) if prm['publish_annotated'] else None
    pub_det = node.create_publisher(Detection2DArray, '/yolo/detections', 10) if Detection2DArray else None
    if pub_det is None:
        log.warn('Không có vision_msgs -> không publish /yolo/detections '
                 '(sudo apt install ros-humble-vision-msgs)')

    # ---------------- kết quả
    out_dir = None
    csv_file = csv_writer = None
    if prm['save_results']:
        out_dir = prm['results_dir'] or os.path.join(lc.results_dir(), 'yolo_' + lc.timestamp())
        os.makedirs(out_dir, exist_ok=True)
        csv_file = open(os.path.join(out_dir, 'detections.csv'), 'w', newline='', encoding='utf-8')
        csv_writer = csv.writer(csv_file)
        csv_writer.writerow(['sim_time', 'frame', 'class', 'conf', 'x1', 'y1', 'x2', 'y2',
                             'robot_x', 'robot_y', 'robot_yaw'])
    stats = DetectionLog()

    show = bool(prm['show_window'])
    if show and not (os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY')):
        log.warn('Không có DISPLAY -> không mở cửa sổ OpenCV (xem /yolo/image_annotated trên RViz2)')
        show = False
    scale = max(1.0, float(prm['window_scale']))

    def show_frame(img):
        nonlocal show
        if not show:
            return
        try:
            if scale != 1.0:
                img = cv2.resize(img, None, fx=scale, fy=scale, interpolation=cv2.INTER_LINEAR)
            cv2.imshow(WINDOW, img)
            cv2.waitKey(1)
        except cv2.error as exc:
            show = False
            log.warn('Không mở được cửa sổ OpenCV ({}). Có thể đang dùng opencv-python-headless: '
                     'pip3 uninstall -y opencv-python-headless && pip3 install --user '
                     '"opencv-python<4.11". Vẫn xem được trên RViz2: /yolo/image_annotated'.format(
                         str(exc).splitlines()[0] if str(exc) else exc))

    def worker():
        last_seq = -1
        last_infer = 0.0
        last_print = 0.0
        last_classes = None
        fps = 0.0
        min_period = 1.0 / prm['max_rate'] if prm['max_rate'] > 0 else 0.0
        warned_no_image = False
        start = time.monotonic()
        waiting = np.zeros((240, 320, 3), dtype=np.uint8)
        last_wait_frame = 0.0
        while not state['stop']:
            with lock:
                msg, seq = state['msg'], state['seq']
            if msg is None or seq == last_seq or time.monotonic() - last_infer < min_period:
                if msg is None:
                    if not warned_no_image and time.monotonic() - start > 15.0:
                        warned_no_image = True
                        log.warn('Chưa nhận ảnh nào trên {} sau 15 s. Kiểm tra: mô phỏng đã chạy? '
                                 '`ros2 topic hz {}`; model:=lite vẫn có camera OAK-D; '
                                 'software_render chậm thì camera cũng chậm.'.format(
                                     prm['image_topic'], prm['image_topic']))
                    if show and time.monotonic() - last_wait_frame > 0.5:
                        last_wait_frame = time.monotonic()
                        img = waiting.copy()
                        cv2.putText(img, 'Dang cho anh camera...', (20, 120), cv2.FONT_HERSHEY_SIMPLEX,
                                    0.6, (255, 255, 255), 1, cv2.LINE_AA)
                        show_frame(img)
                time.sleep(0.01)
                if show:
                    try:
                        cv2.waitKey(1)
                    except cv2.error:
                        pass
                continue
            last_seq = seq
            t_start = time.monotonic()
            last_infer = t_start
            try:
                img = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            except Exception as exc:  # noqa: BLE001
                log.error('cv_bridge không chuyển được ảnh ({}): {}'.format(msg.encoding, exc))
                time.sleep(0.5)
                continue
            try:
                res = model.predict(img, conf=float(prm['conf']), iou=float(prm['iou']),
                                    imgsz=int(prm['imgsz']), device=device, classes=class_ids,
                                    verbose=False)[0]
            except Exception as exc:  # noqa: BLE001
                log.error('YOLO lỗi: {}'.format(exc))
                time.sleep(0.5)
                continue
            dets = []
            if res.boxes is not None and len(res.boxes):
                xyxy = res.boxes.xyxy.cpu().numpy()
                confs = res.boxes.conf.cpu().numpy()
                clss = res.boxes.cls.cpu().numpy().astype(int)
                for (x1, y1, x2, y2), c, k in zip(xyxy, confs, clss):
                    dets.append((names.get(int(k), str(k)), float(c), (float(x1), float(y1),
                                                                        float(x2), float(y2))))
                dets.sort(key=lambda d: -d[1])
            dt = time.monotonic() - t_start
            fps = 0.8 * fps + 0.2 * (1.0 / max(dt, 1e-3)) if fps else 1.0 / max(dt, 1e-3)
            sim_t = lc.stamp_to_sec(msg.header.stamp)
            robot = state['robot']

            hud = ['{} | {:.1f} FPS | {} vat the'.format(model_label, fps, len(dets))]
            if robot is not None:
                hud.append('robot map: ({:.2f}, {:.2f})'.format(robot[0], robot[1]))
            annotated = draw(img.copy(), dets, hud, cv2)

            # publish
            if pub_img is not None:
                out = bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
                out.header = msg.header
                pub_img.publish(out)
            if pub_det is not None:
                arr = Detection2DArray()
                arr.header = msg.header
                for name, conf, (x1, y1, x2, y2) in dets:
                    d = Detection2D()
                    d.header = msg.header
                    d.id = name
                    d.bbox.center.position.x = (x1 + x2) / 2.0
                    d.bbox.center.position.y = (y1 + y2) / 2.0
                    d.bbox.size_x = x2 - x1
                    d.bbox.size_y = y2 - y1
                    hyp = ObjectHypothesisWithPose()
                    hyp.hypothesis.class_id = name
                    hyp.hypothesis.score = conf
                    d.results.append(hyp)
                    arr.detections.append(d)
                pub_det.publish(arr)

            # ghi nhận + in terminal
            seen_now = set()
            for name, conf, box in dets:
                if name in seen_now:
                    continue                                   # mỗi lớp tính 1 lần / khung
                seen_now.add(name)
                new, best = stats.add(name, conf, sim_t, robot)
                if out_dir and (new or best):
                    snap = cv2.resize(annotated, None, fx=scale, fy=scale,
                                      interpolation=cv2.INTER_LINEAR)
                    tag = name.replace(' ', '_')
                    if new:
                        cv2.imwrite(os.path.join(out_dir, 'first_{}.jpg'.format(tag)), snap)
                    cv2.imwrite(os.path.join(out_dir, 'best_{}.jpg'.format(tag)), snap)
                if new:
                    where = '' if robot is None else ' – robot tại ({:.2f}, {:.2f})'.format(robot[0], robot[1])
                    log.info('*** PHÁT HIỆN MỚI: {} (conf {:.2f}){}'.format(name, conf, where))
            if csv_writer is not None:
                for name, conf, (x1, y1, x2, y2) in dets:
                    csv_writer.writerow([round(sim_t, 3), seq, name, round(conf, 3), round(x1), round(y1),
                                         round(x2), round(y2),
                                         '' if robot is None else round(robot[0], 3),
                                         '' if robot is None else round(robot[1], 3),
                                         '' if robot is None else round(robot[2], 3)])
            classes_now = tuple(sorted(seen_now))
            now = time.monotonic()
            if dets and (now - last_print >= prm['print_period'] or classes_now != last_classes):
                last_print = now
                where = '' if robot is None else ' | robot ({:.2f}, {:.2f})'.format(robot[0], robot[1])
                log.info('[t={:.1f}s{}] {} vật thể: {}'.format(sim_t, where, len(dets),
                                                               format_detections(dets)))
            last_classes = classes_now
            show_frame(annotated)

    def safe_worker():
        # Khi node bị tắt (Ctrl+C / SIGTERM từ ros2 launch), context rclpy đóng trong lúc worker có thể
        # đang publish -> RCLError "publisher's context is invalid". Đó là tắt bình thường: thoát êm.
        try:
            worker()
        except Exception as exc:  # noqa: BLE001
            if state['stop'] or not rclpy.ok():
                return
            log.error('Luồng YOLO dừng vì lỗi: {}: {}'.format(type(exc).__name__, exc))
            raise

    th = threading.Thread(target=safe_worker, daemon=True)
    th.start()
    log.info('Đang nghe {} – conf >= {:.2f}, tối đa {:.0f} ảnh/s{}'.format(
        prm['image_topic'], prm['conf'], prm['max_rate'],
        '' if not out_dir else ', lưu kết quả vào ' + out_dir))
    try:
        while rclpy.ok() and not stop_flag['stop'] and th.is_alive():
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        state['stop'] = True
        th.join(timeout=15.0)          # chờ ảnh đang suy luận xong (thường < 0.5 s)
        print('\n================ TỔNG KẾT YOLO ================', flush=True)
        if stats.stats:
            for line in stats.lines():
                print(line, flush=True)
        else:
            print('Chưa phát hiện vật thể nào.', flush=True)
        if out_dir:
            print('Kết quả: {} (detections.csv, first_*.jpg, best_*.jpg)'.format(out_dir), flush=True)
        print('================================================', flush=True)
        if csv_file is not None:
            csv_file.close()
        if show:
            try:
                cv2.destroyAllWindows()
            except Exception:  # noqa: BLE001
                pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    lc.install_ctrl_c_handlers()
    code = lc.run_main(main, interrupted_code=0)
    # Đã đóng file + dừng luồng YOLO + shutdown rclpy. Bỏ qua bước Python huỷ module PyTorch/ultralytics
    # lúc thoát (đo được 2 s, > 5 s khi máy đang chạy Gazebo + Nav2 -> ros2 launch phải SIGTERM).
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(code)
