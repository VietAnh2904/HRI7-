#!/usr/bin/env bash
# tb4_lab04 – kiểm tra môi trường TRƯỚC khi chạy mô phỏng (Gazebo, Nav2, YOLO).
#
#   ros2 run tb4_lab04 doctor.sh        (hoặc: bash scripts/doctor.sh khi chưa build)
#
# Không sửa gì trên máy, chỉ báo PASS / WARN / FAIL và gợi ý cách sửa.
# Mã thoát: 0 = không có lỗi chặn, 1 = có ít nhất một [FAIL].

set -uo pipefail

FAILS=0
WARNS=0
pass() { echo -e "  \e[32m[PASS]\e[0m $*"; }
warn() { echo -e "  \e[33m[WARN]\e[0m $*"; WARNS=$((WARNS + 1)); }
fail() { echo -e "  \e[31m[FAIL]\e[0m $*"; FAILS=$((FAILS + 1)); }
info() { echo -e "  [INFO] $*"; }

IS_WSL=0
if grep -qi microsoft /proc/version 2>/dev/null; then IS_WSL=1; fi

# ---------------------------------------------------------------------------------------------
echo "== 1. Hệ điều hành"
if [[ -r /etc/os-release ]]; then
  # shellcheck disable=SC1091
  . /etc/os-release
  if [[ "${VERSION_ID:-}" == "22.04" ]]; then
    pass "Ubuntu ${VERSION_ID}"
  else
    fail "Cần Ubuntu 22.04 (ROS 2 Humble + Gazebo Fortress), đang là ${PRETTY_NAME:-?}"
  fi
fi
if (( IS_WSL )); then
  info "Đang chạy trong WSL2."
  if [[ -d /mnt/wslg || -n "${WAYLAND_DISPLAY:-}" ]]; then
    pass "Có WSLg (hiển thị cửa sổ Linux trên Windows)"
  else
    fail "Không thấy WSLg -> cần Windows 11 hoặc Windows 10 22H2 + 'wsl --update' (xem README mục 1.2)"
  fi
fi

# ---------------------------------------------------------------------------------------------
echo "== 2. ROS 2"
if [[ "${ROS_DISTRO:-}" == "humble" ]]; then
  pass "ROS_DISTRO=humble"
else
  fail "Chưa source ROS 2 Humble (ROS_DISTRO='${ROS_DISTRO:-}') -> source /opt/ros/humble/setup.bash"
fi
if ! command -v ros2 >/dev/null 2>&1; then
  fail "Không có lệnh ros2 -> sudo apt install ros-humble-desktop"
fi
# Anaconda/Miniconda đang bật: python3 trỏ sang Python của conda -> rclpy/cv_bridge (build cho Python 3.10
# hệ thống) không import được, pip cài YOLO vào nhầm môi trường.
PY3="$(command -v python3 || true)"
if [[ -n "${CONDA_PREFIX:-}" || "$PY3" == *conda* || "$PY3" == *miniforge* ]]; then
  fail "Đang bật conda (python3 = $PY3) -> chạy 'conda deactivate' (và 'conda config --set auto_activate_base false'), mở terminal mới"
elif [[ -n "$PY3" && "$PY3" != "/usr/bin/python3" ]]; then
  warn "python3 = $PY3 (không phải /usr/bin/python3 của Ubuntu) – ROS 2 Humble cần Python 3.10 hệ thống"
else
  pass "python3 = $PY3 ($(python3 --version 2>&1))"
fi

# ---------------------------------------------------------------------------------------------
echo "== 3. Gói ROS cần thiết (và có bị workspace khác 'ghi đè' không)"
REQUIRED="turtlebot4_ignition_bringup turtlebot4_navigation turtlebot4_description turtlebot4_viz \
irobot_create_nodes irobot_create_msgs irobot_create_ignition_bringup ros_gz_bridge ros_gz_sim ros_ign_gazebo \
slam_toolbox nav2_bringup nav2_map_server nav2_amcl nav2_rviz_plugins rviz2 teleop_twist_keyboard \
topic_tools tf2_ros_py sensor_msgs_py nav2_simple_commander cv_bridge vision_msgs"
if command -v python3 >/dev/null 2>&1; then
  # Một lần gọi Python cho tất cả gói (nhanh hơn gọi `ros2 pkg prefix` từng gói).
  PKG_REPORT="$(python3 - $REQUIRED <<'PY' 2>/dev/null
import sys
try:
    from ament_index_python.packages import get_package_prefix, PackageNotFoundError
except ImportError:
    print('NOAMENT')
    sys.exit(0)
for p in sys.argv[1:]:
    try:
        print('OK', p, get_package_prefix(p))
    except PackageNotFoundError:
        print('MISSING', p, '-')
PY
)"
  if [[ "$PKG_REPORT" == "NOAMENT" || -z "$PKG_REPORT" ]]; then
    fail "Không đọc được danh sách gói ROS (chưa source /opt/ros/humble/setup.bash?)"
  else
    MISSING=""
    OVERRIDDEN=0
    while read -r st p prefix; do
      if [[ "$st" == "MISSING" ]]; then
        MISSING="$MISSING ros-humble-${p//_/-}"
        fail "$p chưa cài"
      elif [[ "$prefix" != "/opt/ros/humble" ]]; then
        # Gói hệ thống bị một workspace khác che: nguồn gây lỗi rất khó hiểu (vd. ros_gz build cho
        # Gazebo Harmonic từ một workspace UR/khác).
        warn "$p đang lấy từ '$prefix' thay vì /opt/ros/humble (workspace khác ghi đè gói hệ thống)"
        OVERRIDDEN=1
      else
        pass "$p"
      fi
    done <<< "$PKG_REPORT"
    if [[ -n "$MISSING" ]]; then
      echo "        -> sudo apt update && sudo apt install -y$MISSING"
    fi
    if (( OVERRIDDEN )); then
      echo "        -> Nếu bạn KHÔNG cố ý build các gói trên từ source, đó rất có thể là nguyên nhân lỗi:"
      echo "           bỏ dòng 'source <workspace đó>/install/setup.bash' khỏi ~/.bashrc rồi mở terminal mới."
    fi
  fi
fi
if command -v ros2 >/dev/null 2>&1 && ros2 pkg prefix tb4_lab04 >/dev/null 2>&1; then
  pass "tb4_lab04 (đã build và source workspace: $(ros2 pkg prefix tb4_lab04))"
else
  warn "tb4_lab04 chưa thấy -> cd ~/workspaces/tb4_gazebo && colcon build && source install/setup.bash"
fi

echo "== 4. Workspace đang được source (AMENT_PREFIX_PATH)"
OTHER=""
IFS=':' read -r -a PREFIXES <<< "${AMENT_PREFIX_PATH:-}"
for p in "${PREFIXES[@]}"; do
  [[ -z "$p" ]] && continue
  case "$p" in
    /opt/ros/humble) ;;
    */install/tb4_lab04|*/install) ;;
    *) OTHER="$OTHER $p" ;;
  esac
done
if [[ -n "$OTHER" ]]; then
  warn "Đang source thêm workspace khác:$OTHER"
  echo "        Nếu gặp lỗi lạ, mở terminal mới KHÔNG source các workspace đó (xem ~/.bashrc)."
else
  pass "Chỉ có /opt/ros/humble và workspace của bài"
fi

# ---------------------------------------------------------------------------------------------
echo "== 5. Ignition Gazebo Fortress"
if command -v ign >/dev/null 2>&1; then
  V="$(timeout 20 ign gazebo --versions 2>/dev/null | head -n 1)"
  if [[ "$V" == 6.* ]]; then pass "ign gazebo $V (Fortress)"; else warn "ign gazebo --versions = '${V:-không rõ}' (Fortress là 6.x)"; fi
else
  fail "Không có lệnh 'ign' -> sudo apt install ignition-fortress (xem README bước cài đặt)"
fi
if [[ -n "${GZ_VERSION:-}${IGNITION_VERSION:-}" ]]; then
  warn "Đang đặt GZ_VERSION/IGNITION_VERSION='${GZ_VERSION:-}${IGNITION_VERSION:-}' – TurtleBot 4 Humble cần Fortress; hãy unset nếu không chắc"
fi

# ---------------------------------------------------------------------------------------------
echo "== 6. Mạng ROS 2"
if [[ "${ROS_LOCALHOST_ONLY:-0}" == "1" ]]; then
  pass "ROS_LOCALHOST_ONLY=1"
elif [[ -n "${ROS_DOMAIN_ID:-}" && "${ROS_DOMAIN_ID}" != "0" ]]; then
  pass "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
else
  warn "Đang dùng domain mặc định 0 trên mạng chung -> export ROS_LOCALHOST_ONLY=1 (thêm vào ~/.bashrc)"
fi
if [[ "${RMW_IMPLEMENTATION:-}" == "rmw_cyclonedds_cpp" && "${ROS_LOCALHOST_ONLY:-0}" == "1" ]]; then
  warn "CycloneDDS + ROS_LOCALHOST_ONLY=1 trên Humble hay lỗi 'failed to create domain' -> unset RMW_IMPLEMENTATION"
fi

# ---------------------------------------------------------------------------------------------
echo "== 7. Tiến trình còn sót"
if pgrep -u "$(id -u)" -f 'ign gazebo|gz sim' >/dev/null 2>&1; then
  warn "Còn Gazebo đang chạy -> ros2 run tb4_lab04 kill_sim.sh"
else
  pass "Không có Gazebo cũ"
fi

# ---------------------------------------------------------------------------------------------
echo "== 8. Đồ hoạ"
if [[ -z "${DISPLAY:-}" && -z "${WAYLAND_DISPLAY:-}" ]]; then
  fail "Không có DISPLAY/WAYLAND_DISPLAY: Gazebo GUI và RViz2 sẽ không mở được"
else
  pass "Có màn hình (${DISPLAY:-}${WAYLAND_DISPLAY:+ wayland:$WAYLAND_DISPLAY})"
fi
if [[ "${LIBGL_ALWAYS_SOFTWARE:-}" == "1" || "${LIBGL_ALWAYS_SOFTWARE:-}" == "true" ]]; then
  info "LIBGL_ALWAYS_SOFTWARE=${LIBGL_ALWAYS_SOFTWARE} -> đang ép render bằng CPU (đúng nhưng chậm)"
fi
if command -v glxinfo >/dev/null 2>&1; then
  R="$(timeout 20 glxinfo -B 2>/dev/null | awk -F': ' '/OpenGL renderer/ {print $2}')"
  if [[ -z "$R" ]]; then
    warn "glxinfo không lấy được OpenGL renderer (driver đồ hoạ có vấn đề?)"
  elif [[ "$R" == *D3D12* ]]; then
    warn "OpenGL: $R (GPU qua WSL2). Nếu Gazebo crash hoặc check_sensors.py báo LiDAR/cliff sai -> thêm software_render:=true vào lệnh launch"
  elif [[ "$R" == *llvmpipe* || "$R" == *softpipe* ]]; then
    warn "OpenGL: $R (render bằng CPU) -> Gazebo chạy được nhưng chậm (RTF thấp)"
  else
    pass "OpenGL: $R"
  fi
else
  info "Bỏ qua kiểm tra GPU – cài 'mesa-utils' để có glxinfo: sudo apt install -y mesa-utils"
fi

# ---------------------------------------------------------------------------------------------
echo "== 9. RAM / CPU"
MEM_GB=$(awk '/MemTotal/ {printf "%.0f", $2/1024/1024}' /proc/meminfo)
if (( MEM_GB >= 8 )); then
  pass "RAM ${MEM_GB} GB"
elif (( IS_WSL )); then
  warn "RAM của WSL2 chỉ ${MEM_GB} GB -> tăng trong %UserProfile%\\.wslconfig (memory=8GB), rồi 'wsl --shutdown'"
else
  warn "RAM ${MEM_GB} GB (< 8 GB) – Gazebo + Nav2 có thể rất chậm"
fi
CPUS=$(nproc)
if (( CPUS >= 4 )); then pass "${CPUS} CPU"; else warn "${CPUS} CPU (< 4) – có thể rất chậm"; fi

# ---------------------------------------------------------------------------------------------
echo "== 10. Bản đồ (cần cho Phần 3, 4)"
MAPDIR=""
if command -v ros2 >/dev/null 2>&1 && PREFIX="$(ros2 pkg prefix tb4_lab04 2>/dev/null)"; then
  MARKER="$PREFIX/share/tb4_lab04/source_dir.txt"
  [[ -f "$MARKER" ]] && MAPDIR="$(head -n 1 "$MARKER")/maps"
fi
if [[ -n "$MAPDIR" && -f "$MAPDIR/map.yaml" && -f "$MAPDIR/map.pgm" ]]; then
  pass "Có $MAPDIR/map.yaml + map.pgm"
else
  info "Chưa có maps/map.yaml + map.pgm -> làm Phần 2 rồi chạy: ros2 run tb4_lab04 save_map.sh"
fi

# ---------------------------------------------------------------------------------------------
echo "== 11. Python cho YOLO (ultralytics, OpenCV, NumPy, cv_bridge)"
if command -v python3 >/dev/null 2>&1; then
  PY_REPORT="$(python3 - <<'PY' 2>&1
import importlib, sys
def ver(name):
    try:
        m = importlib.import_module(name)
        return getattr(m, '__version__', '?'), getattr(m, '__file__', '?')
    except Exception as exc:
        return None, '{}: {}'.format(type(exc).__name__, str(exc).splitlines()[0][:150] if str(exc) else '')
for name in ('numpy', 'cv2', 'torch', 'ultralytics'):
    v, where = ver(name)
    print('MOD', name, v or '-', where, flush=True)
try:
    import numpy
    if int(numpy.__version__.split('.')[0]) >= 2:
        # cv_bridge (build với NumPy 1.x) làm tiến trình segfault với NumPy 2 -> không thử
        raise RuntimeError('NumPy {} (cần < 2)'.format(numpy.__version__))
    from cv_bridge import CvBridge
    import numpy as np
    from sensor_msgs.msg import Image
    b = CvBridge()
    m = Image(height=2, width=2, encoding='rgb8', step=6, data=bytes(12))
    b.imgmsg_to_cv2(m, desired_encoding='bgr8')
    print('BRIDGE OK', flush=True)
except Exception as exc:
    print('BRIDGE FAIL', type(exc).__name__, str(exc).splitlines()[0][:200] if str(exc) else '', flush=True)
try:
    import cv2
    info = cv2.getBuildInformation()
    gui = [l.split(':', 1)[1].strip() for l in info.splitlines() if l.strip().startswith('GUI:')]
    print('CV2GUI', gui[0] if gui else '?')
except Exception:
    print('CV2GUI ?')
PY
)"
  NP_VER="$(awk '$1=="MOD" && $2=="numpy" {print $3}' <<< "$PY_REPORT")"
  while read -r tag name v where; do
    [[ "$tag" != "MOD" ]] && continue
    if [[ "$v" == "-" ]]; then
      case "$name" in
        numpy) fail "numpy chưa cài -> sudo apt install python3-numpy" ;;
        cv2) fail "OpenCV (cv2) chưa có -> pip3 install --user \"opencv-python<4.11\"" ;;
        torch|ultralytics) warn "$name chưa cài – chỉ cần cho node YOLO (Bài 5): bash scripts/install_yolo_deps.sh" ;;
      esac
    elif [[ "$name" == numpy && "$v" == 2.* ]]; then
      :   # báo FAIL riêng ngay bên dưới
    else
      pass "$name $v ($where)"
    fi
  done <<< "$PY_REPORT"
  if [[ "$NP_VER" == 2.* ]]; then
    fail "NumPy $NP_VER: cv_bridge của ROS 2 Humble (apt) build với NumPy 1.x -> lỗi khi chuyển ảnh. Sửa: pip3 install --user \"numpy<2\" \"opencv-python<4.11\""
  fi
  if grep -q '^BRIDGE OK' <<< "$PY_REPORT"; then
    pass "cv_bridge chuyển được sensor_msgs/Image -> OpenCV"
  else
    fail "cv_bridge lỗi: $(grep '^BRIDGE' <<< "$PY_REPORT" | cut -c13-) -> sudo apt install ros-humble-cv-bridge; nếu lỗi NumPy: pip3 install --user \"numpy<2\""
  fi
  GUI="$(awk '$1=="CV2GUI" {$1=""; print substr($0,2)}' <<< "$PY_REPORT")"
  if [[ "$GUI" == "NONE" || -z "$GUI" ]]; then
    warn "OpenCV không có GUI (opencv-python-headless?) -> cửa sổ YOLO không mở được; vẫn xem trên RViz2. Sửa: pip3 uninstall -y opencv-python-headless && pip3 install --user \"opencv-python<4.11\""
  else
    pass "OpenCV GUI: $GUI"
  fi
fi
WDIR="${HOME}/.cache/tb4_lab04/models"
if ls "$WDIR"/*.pt >/dev/null 2>&1; then
  pass "Model YOLO đã tải: $(cd "$WDIR" && ls *.pt | tr '\n' ' ')"
else
  info "Chưa có model YOLO trong $WDIR (lần chạy đầu yolo_detector.py tự tải – cần Internet)"
fi

echo "== 12. Model Gazebo Fuel cho vật thể (config/objects.yaml)"
if command -v ros2 >/dev/null 2>&1 && ros2 pkg prefix tb4_lab04 >/dev/null 2>&1; then
  LIST="$(timeout 30 ros2 run tb4_lab04 spawn_objects.py --list 2>&1)"
  NOK="$(grep -c 'có trong cache' <<< "$LIST")"
  NMISS="$(grep -c 'CHƯA tải' <<< "$LIST")"
  if (( NMISS == 0 && NOK > 0 )); then
    pass "$NOK model vật thể đã có trong cache Fuel"
  else
    warn "$NMISS model vật thể chưa tải -> ros2 run tb4_lab04 spawn_objects.py --download-only (cần Internet)"
  fi
fi

echo "== 13. Bài 6 – cử chỉ tay (MediaPipe, model, webcam)"
if command -v python3 >/dev/null 2>&1; then
  MP_REPORT="$(python3 - <<'PY' 2>&1
try:
    import mediapipe as mp
    from mediapipe.tasks.python import vision  # noqa: F401
    print('MP', getattr(mp, '__version__', '?'))
except Exception as exc:
    print('MPFAIL', type(exc).__name__, str(exc).splitlines()[0][:150] if str(exc) else '')
PY
)"
  if grep -q '^MP ' <<< "$MP_REPORT"; then
    pass "mediapipe $(awk '$1=="MP"{print $2}' <<< "$MP_REPORT") (Gesture Recognizer)"
  else
    fail "mediapipe chưa dùng được: $(cut -c8- <<< "$MP_REPORT") -> bash scripts/install_gesture_deps.sh"
  fi
fi
GDIR=""
if command -v ros2 >/dev/null 2>&1 && PREFIX="$(ros2 pkg prefix tb4_lab04 2>/dev/null)"; then
  GDIR="$PREFIX/share/tb4_lab04"
fi
if [[ -n "$GDIR" && -s "$GDIR/models/gesture_recognizer.task" ]]; then
  pass "Model gesture_recognizer.task có sẵn"
elif [[ -s "$HOME/.cache/tb4_lab04/models/gesture_recognizer.task" ]]; then
  pass "Model gesture_recognizer.task trong ~/.cache/tb4_lab04/models"
else
  warn "Chưa thấy models/gesture_recognizer.task – node sẽ tự tải (cần Internet) hoặc build lại package"
fi
if [[ -n "$GDIR" && -f "$GDIR/worlds/gesture_arena.sdf" ]]; then
  pass "World gesture_arena đã install"
elif [[ -n "$GDIR" ]]; then
  fail "Thiếu share/tb4_lab04/worlds/gesture_arena.sdf -> colcon build --symlink-install --packages-select tb4_lab04"
fi
if ls /dev/video* >/dev/null 2>&1; then
  pass "Webcam: $(ls /dev/video* | tr '\n' ' ')"
  id -nG 2>/dev/null | grep -qw video || warn "User chưa thuộc nhóm video – nếu không mở được webcam: sudo usermod -aG video \$USER (đăng nhập lại)"
else
  warn "Không có /dev/video* – webcam chưa được nhận (máy ảo/WSL2: chuyển webcam vào trong)"
fi

echo
if (( FAILS == 0 )); then
  echo -e "\e[32mKết luận: không có lỗi chặn ($WARNS cảnh báo).\e[0m"
  exit 0
fi
echo -e "\e[31mKết luận: $FAILS lỗi cần sửa trước khi chạy ($WARNS cảnh báo).\e[0m"
exit 1
