#!/usr/bin/env python3
"""Bài 6 – Phần 1: đọc và hiển thị webcam bằng OpenCV (không cần ROS, không cần MediaPipe).

  ros2 run tb4_lab04 webcam_test.py            # webcam 0 (laptop)
  ros2 run tb4_lab04 webcam_test.py 1          # webcam USB
  python3 scripts/webcam_test.py --list        # dò các /dev/video* mở được
  python3 scripts/webcam_test.py 0 --seconds 5 --no-window   # chỉ đo FPS (máy không màn hình)

Cửa sổ hiển thị độ phân giải + FPS đo được. q / Esc / nút X để thoát.
Thoát mã 0 nếu đọc được khung hình, 1 nếu không mở được camera.
"""

import argparse
import glob
import sys
import time

WINDOW = 'Webcam test (OpenCV)'


def open_camera(cv2, src, width, height):
    if isinstance(src, int) and sys.platform.startswith('linux'):
        cap = cv2.VideoCapture(src, cv2.CAP_V4L2)
        if cap.isOpened():
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            return cap
        cap.release()
    cap = cv2.VideoCapture(src)
    if cap.isOpened():
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        return cap
    cap.release()
    return None


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('source', nargs='?', default='0', help='chỉ số webcam (0, 1, ...) hoặc đường dẫn video')
    ap.add_argument('--width', type=int, default=640)
    ap.add_argument('--height', type=int, default=480)
    ap.add_argument('--seconds', type=float, default=0.0, help='tự thoát sau N giây (0 = chạy mãi)')
    ap.add_argument('--no-window', action='store_true', help='không mở cửa sổ (chỉ in FPS)')
    ap.add_argument('--list', action='store_true', help='liệt kê camera mở được rồi thoát')
    args, _ = ap.parse_known_args(argv)   # bỏ qua --ros-args khi chạy bằng ros2 run

    try:
        import cv2
    except ImportError as exc:
        print('Không import được OpenCV ({}). Cài: pip3 install --user "opencv-python<4.11" "numpy<2"'.format(exc))
        return 1

    import os
    cur = os.environ.get('QT_QPA_FONTDIR')     # xem fix_qt_fonts() trong gesture_recognition_node.py
    if not (cur and os.path.isdir(cur)) and os.path.isdir('/usr/share/fonts/truetype/dejavu'):
        os.environ['QT_QPA_FONTDIR'] = '/usr/share/fonts/truetype/dejavu'

    if args.list:
        devs = sorted(glob.glob('/dev/video*'))
        print('Thiết bị: {}'.format(', '.join(devs) or '(không có /dev/video*)'))
        for i in range(max(4, len(devs))):
            cap = open_camera(cv2, i, args.width, args.height)
            ok = False
            if cap is not None:
                ok, frame = cap.read()
                cap.release()
            print('  camera {}: {}'.format(i, 'OK {}x{}'.format(frame.shape[1], frame.shape[0]) if ok else '-'))
        return 0

    src = int(args.source) if args.source.lstrip('-').isdigit() else args.source
    cap = open_camera(cv2, src, args.width, args.height)
    if cap is None:
        print('KHÔNG mở được camera {}. Kiểm tra: ls /dev/video* ; thử số khác (--list); '
              'sudo usermod -aG video $USER (đăng xuất/đăng nhập lại); đóng Zoom/Meet/Cheese; '
              'máy ảo: bật chuyển webcam vào máy ảo (VirtualBox: Devices > Webcams; WSL2: usbipd).'.format(src))
        return 1
    show = not args.no_window
    if show:
        try:
            cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
        except cv2.error as exc:
            print('Không mở được cửa sổ ({}) – chạy không hiển thị'.format(exc))
            show = False
    t0 = time.monotonic()
    n, fps, last = 0, 0.0, t0
    seen_visible = False
    code = 0
    try:
        while True:
            ok, frame = cap.read()
            now = time.monotonic()
            if not ok:
                if not isinstance(src, int) and n > 0:
                    print('Hết video.')
                    break
                if now - last > 3.0:
                    print('Camera không trả khung hình trong 3 s')
                    code = 1
                    break
                continue
            n += 1
            dt = now - last
            last = now
            if dt > 0:
                fps = 1.0 / dt if fps == 0 else 0.9 * fps + 0.1 / dt
            h, w = frame.shape[:2]
            if show:
                frame = cv2.flip(frame, 1)
                cv2.putText(frame, '{}x{}  {:.1f} FPS  (q: thoat)'.format(w, h, fps), (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2, cv2.LINE_AA)
                cv2.imshow(WINDOW, frame)
                key = cv2.waitKey(1) & 0xFF
                if key in (27, ord('q')):
                    break
                vis = cv2.getWindowProperty(WINDOW, cv2.WND_PROP_VISIBLE)
                if vis >= 1:
                    seen_visible = True
                elif seen_visible:
                    break
            if n % 60 == 0:
                print('{} khung, {}x{}, {:.1f} FPS'.format(n, w, h, fps), flush=True)
            if args.seconds > 0 and now - t0 >= args.seconds:
                break
    except KeyboardInterrupt:
        pass
    finally:
        cap.release()
        if show:
            cv2.destroyAllWindows()
    print('Kết quả: {} khung hình trong {:.1f} s ({})'.format(n, time.monotonic() - t0,
                                                              'ĐẠT' if n > 0 else 'KHÔNG ĐẠT'))
    return code if n > 0 else 1


if __name__ == '__main__':
    sys.exit(main())
