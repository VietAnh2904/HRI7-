#!/usr/bin/env bash
# Cài thư viện YOLO cho Lab 04 trên Ubuntu 22.04 + ROS 2 Humble (không dùng sudo cho phần pip).
# Chạy:  bash scripts/install_yolo_deps.sh        (thêm --gpu để cài PyTorch bản mặc định/CUDA)
# Có thể chạy lại nhiều lần an toàn.
set -u
GPU=0; [ "${1:-}" = "--gpu" ] && GPU=1

say(){ printf '\n\033[1;36m== %s ==\033[0m\n' "$*"; }
die(){ printf '\033[1;31m[LỖI]\033[0m %s\n' "$*"; exit 1; }

say "0. Kiểm tra môi trường"
if [ -n "${CONDA_DEFAULT_ENV:-}" ] || [ -n "${VIRTUAL_ENV:-}" ]; then
  die "Đang trong môi trường conda/venv (${CONDA_DEFAULT_ENV:-$VIRTUAL_ENV}). Chạy 'conda deactivate' (nhiều lần nếu cần) rồi mở terminal mới."
fi
PY=/usr/bin/python3
[ -x "$PY" ] || die "Không thấy /usr/bin/python3"
"$PY" --version
[ -f /opt/ros/humble/setup.bash ] || die "Chưa cài ROS 2 Humble (/opt/ros/humble)."

say "1. Gói apt (cần sudo)"
sudo apt-get update -y && sudo apt-get install -y \
  python3-pip python3-opencv ros-humble-cv-bridge ros-humble-vision-msgs \
  ros-humble-image-transport python3-yaml python3-matplotlib || die "apt thất bại"

say "2. NumPy 1.x + OpenCV (có GUI) — GHIM phiên bản để cv_bridge không hỏng"
"$PY" -m pip install --user -U pip >/dev/null 2>&1
"$PY" -m pip install --user "numpy<2" "opencv-python<4.11" || die "cài numpy/opencv thất bại"

say "3. PyTorch"
if [ $GPU -eq 1 ]; then
  "$PY" -m pip install --user torch torchvision || die "cài torch thất bại"
else
  "$PY" -m pip install --user torch torchvision --index-url https://download.pytorch.org/whl/cpu \
    || die "cài torch (CPU) thất bại"
fi

say "4. Ultralytics (YOLO) — vẫn giữ numpy<2, opencv<4.11"
"$PY" -m pip install --user ultralytics "numpy<2" "opencv-python<4.11" || die "cài ultralytics thất bại"
# Nếu ultralytics kéo opencv-python-headless thì cửa sổ OpenCV không mở được -> gỡ
"$PY" -m pip uninstall -y opencv-python-headless >/dev/null 2>&1
"$PY" -m pip install --user --force-reinstall --no-deps "opencv-python<4.11" >/dev/null 2>&1

say "5. Tải sẵn trọng số yolo11n.pt (cần Internet)"
mkdir -p "$HOME/.cache/tb4_lab04/models"
( cd "$HOME/.cache/tb4_lab04/models" && "$PY" - <<'PYEOF'
try:
    from ultralytics import YOLO
    YOLO("yolo11n.pt")
    print("OK: yolo11n.pt")
except Exception as e:
    print("CẢNH BÁO: chưa tải được trọng số:", e)
PYEOF
)

say "6. Kiểm tra cuối"
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
"$PY" - <<'PYEOF'
import sys
ok = True
def chk(name, fn):
    global ok
    try: print(f"[PASS] {name}: {fn()}")
    except Exception as e:
        ok = False; print(f"[FAIL] {name}: {type(e).__name__}: {e}")
import numpy; chk("numpy (phải 1.x)", lambda: numpy.__version__ if numpy.__version__.startswith("1.") else (_ for _ in ()).throw(RuntimeError("đang là NumPy 2.x")))
chk("cv2", lambda: __import__("cv2").__version__)
chk("torch", lambda: __import__("torch").__version__)
chk("ultralytics", lambda: __import__("ultralytics").__version__)
def bridge():
    from cv_bridge import CvBridge
    import numpy as np
    CvBridge().cv2_to_imgmsg(np.zeros((4, 4, 3), np.uint8), "bgr8"); return "CvBridge OK"
chk("cv_bridge", bridge)
chk("vision_msgs", lambda: __import__("vision_msgs.msg").__name__)
sys.exit(0 if ok else 1)
PYEOF
rc=$?
[ $rc -eq 0 ] && say "XONG — mọi thứ đã sẵn sàng" || say "CÓ LỖI ở trên — xem README mục 'Xử lý sự cố' hoặc gửi log cho tôi"
exit $rc
