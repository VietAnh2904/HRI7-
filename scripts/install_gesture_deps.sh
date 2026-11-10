#!/usr/bin/env bash
# Bài 6 – cài thư viện nhận diện cử chỉ (OpenCV + MediaPipe) trên Ubuntu 22.04 + ROS 2 Humble.
# Chạy trong thư mục gói:  bash scripts/install_gesture_deps.sh      (chạy lại nhiều lần vẫn an toàn)
#
# Vì sao ghim phiên bản (đã kiểm thử: Python 3.10, NumPy 1.26.4, OpenCV 4.10.0.84, MediaPipe 0.10.21):
#   * `pip install mediapipe` bản mới kéo opencv-contrib-python 4.12+ -> bắt buộc NumPy 2.x -> cv_bridge
#     (apt, build với NumPy 1.x) và node YOLO của bài trước hỏng. Ghim "numpy<2" + OpenCV < 4.11.
#   * opencv-python (YOLO) và opencv-contrib-python (MediaPipe) cùng ghi vào thư mục cv2 -> giữ CÙNG
#     phiên bản 4.10.x để hai gói không phá nhau. KHÔNG gỡ một trong hai (sẽ xoá mất cv2).
set -u

say(){ printf '\n\033[1;36m== %s ==\033[0m\n' "$*"; }
die(){ printf '\033[1;31m[LỖI]\033[0m %s\n' "$*"; exit 1; }
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_DIR="$(dirname "$HERE")"
MP_VERSION="${MP_VERSION:-0.10.21}"

say "0. Kiểm tra môi trường"
if [ -n "${CONDA_DEFAULT_ENV:-}" ] || [ -n "${VIRTUAL_ENV:-}" ]; then
  die "Đang trong conda/venv (${CONDA_DEFAULT_ENV:-${VIRTUAL_ENV:-}}). Chạy 'conda deactivate' rồi mở terminal mới."
fi
PY=/usr/bin/python3
[ -x "$PY" ] || die "Không thấy /usr/bin/python3"
"$PY" --version
[ -f /opt/ros/humble/setup.bash ] || echo "CẢNH BÁO: không thấy /opt/ros/humble – vẫn cài thư viện Python."

say "1. Gói apt (cần sudo): pip, v4l-utils (dò webcam), matplotlib"
sudo apt-get update -y && sudo apt-get install -y python3-pip v4l-utils python3-matplotlib python3-yaml \
  || die "apt thất bại"

say "2. MediaPipe ${MP_VERSION} + NumPy 1.x + OpenCV 4.10 (có GUI)"
"$PY" -m pip install --user "mediapipe==${MP_VERSION}" "numpy<2" "opencv-python<4.11" "opencv-contrib-python<4.11" \
  || die "pip install thất bại (cần Internet tới pypi.org)"
# opencv-python-headless (do gói khác kéo về) làm cửa sổ OpenCV không mở được
if "$PY" -m pip show opencv-python-headless >/dev/null 2>&1; then
  "$PY" -m pip uninstall -y opencv-python-headless
  "$PY" -m pip install --user --force-reinstall --no-deps "opencv-contrib-python<4.11"
fi

say "3. Kiểm tra"
MODEL="$PKG_DIR/models/gesture_recognizer.task"
"$PY" - "$MODEL" <<'PYEOF'
import os, sys
ok = True
def chk(name, fn):
    global ok
    try:
        print("[PASS] {}: {}".format(name, fn()))
    except Exception as e:
        ok = False
        print("[FAIL] {}: {}: {}".format(name, type(e).__name__, e))
import numpy
def np_ver():
    if not numpy.__version__.startswith("1."):
        raise RuntimeError("NumPy {} (cần 1.x cho cv_bridge của Humble)".format(numpy.__version__))
    return numpy.__version__
chk("numpy", np_ver)
def cv():
    import cv2
    info = cv2.getBuildInformation()
    gui = [l.strip() for l in info.splitlines() if l.strip().startswith(("GTK", "QT"))]
    if not any("YES" in g for g in gui):
        raise RuntimeError("OpenCV {} KHÔNG có GUI (bản headless?)".format(cv2.__version__))
    return cv2.__version__
chk("cv2 (có GUI)", cv)
chk("mediapipe", lambda: __import__("mediapipe").__version__)
def recog():
    import numpy as np
    import mediapipe as mp
    from mediapipe.tasks.python import BaseOptions, vision
    model = sys.argv[1]
    if not os.path.isfile(model):
        return "bỏ qua (chưa có {} – node sẽ tự tải)".format(model)
    opt = vision.GestureRecognizerOptions(base_options=BaseOptions(model_asset_path=model),
                                          running_mode=vision.RunningMode.IMAGE)
    r = vision.GestureRecognizer.create_from_options(opt)
    res = r.recognize(mp.Image(image_format=mp.ImageFormat.SRGB, data=np.zeros((240, 320, 3), np.uint8)))
    r.close()
    return "Gesture Recognizer chạy được (ảnh đen -> {} tay)".format(len(res.hand_landmarks))
chk("MediaPipe Gesture Recognizer", recog)
def bridge():
    try:
        from cv_bridge import CvBridge
    except ImportError:
        return "bỏ qua (chưa source ROS)"
    import numpy as np
    CvBridge().cv2_to_imgmsg(np.zeros((4, 4, 3), np.uint8), "bgr8")
    return "CvBridge OK (NumPy không xung đột)"
chk("cv_bridge", bridge)
sys.exit(0 if ok else 1)
PYEOF
rc=$?

say "4. Webcam"
ls /dev/video* 2>/dev/null || echo "Không có /dev/video* – máy ảo/WSL2 cần chuyển webcam vào trong."
command -v v4l2-ctl >/dev/null && v4l2-ctl --list-devices 2>/dev/null | head -20
id -nG | grep -qw video || echo "Gợi ý: sudo usermod -aG video \$USER (rồi đăng xuất / đăng nhập lại)"

[ $rc -eq 0 ] && say "XONG – thử: python3 scripts/webcam_test.py" || say "CÓ LỖI ở trên – xem README Bài 6 mục Xử lý sự cố"
exit $rc
