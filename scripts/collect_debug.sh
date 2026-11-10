#!/usr/bin/env bash
# tb4_lab04 – gom thông tin chẩn đoán vào MỘT file để gửi cho người hỗ trợ.
#
#   ros2 run tb4_lab04 collect_debug.sh            # chạy khi đang gặp lỗi (launch vẫn mở càng tốt)
#
# Tạo file ~/tb4_debug_<thời_gian>.txt gồm: kết quả doctor.sh, biến môi trường ROS/Gazebo,
# phiên bản gói apt, OpenGL renderer, danh sách node/topic, tần số /clock /scan /odom (nếu sim đang
# chạy), và 300 dòng cuối của log launch gần nhất. Không gửi đi đâu cả – bạn tự gửi file đó.

set -uo pipefail

OUT="${HOME}/tb4_debug_$(date +%Y%m%d_%H%M%S).txt"
SCRIPT_DIR="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)"

section() { echo; echo "################ $* ################"; }

{
  echo "tb4_lab04 debug report – $(date)"

  section "doctor.sh"
  bash "$SCRIPT_DIR/doctor.sh" 2>&1 | sed 's/\x1b\[[0-9;]*m//g'

  section "uname / os-release / WSL"
  uname -a
  grep -E '^(PRETTY_NAME|VERSION_ID)=' /etc/os-release 2>/dev/null
  grep -i microsoft /proc/version 2>/dev/null && echo "(WSL)"

  section "Biến môi trường liên quan"
  env | grep -E '^(ROS_|RMW_|AMENT_PREFIX_PATH|COLCON_|GZ_|IGN_|IGNITION_|LIBGL_|MESA_|GALLIUM_|__NV_|__GLX_|DISPLAY|WAYLAND_DISPLAY|LD_LIBRARY_PATH|PYTHONPATH)' | sort

  section "Phiên bản gói"
  dpkg-query -W -f='${Package} ${Version}\n' 'ros-humble-turtlebot4*' 'ros-humble-irobot-create*' \
    'ros-humble-nav2-bringup' 'ros-humble-nav2-amcl' 'ros-humble-slam-toolbox' 'ros-humble-ros-gz*' \
    'ros-humble-ros-ign*' 'ros-humble-topic-tools' 'ignition-fortress' 'libignition-gazebo6*' \
    'ros-humble-rmw-fastrtps-cpp' 2>/dev/null
  command -v ign >/dev/null && echo "ign gazebo --versions: $(timeout 20 ign gazebo --versions 2>&1 | head -1)"
  command -v gz >/dev/null && echo "gz (khác Fortress?): $(timeout 20 gz sim --versions 2>&1 | head -1)"

  section "OpenGL"
  if command -v glxinfo >/dev/null; then timeout 20 glxinfo -B 2>&1 | head -20; else echo "không có glxinfo (sudo apt install mesa-utils)"; fi
  command -v nvidia-smi >/dev/null && timeout 20 nvidia-smi -L 2>&1

  section "Tài nguyên"
  nproc; free -h

  section "Tiến trình Gazebo / ROS đang chạy"
  pgrep -a -f 'ign gazebo|gz sim|ros2 launch|--ros-args' 2>/dev/null | cut -c1-200 | head -60

  if command -v ros2 >/dev/null 2>&1; then
    section "ros2 node list / topic list"
    timeout 30 ros2 node list --no-daemon 2>&1 | head -80
    echo "---"
    timeout 30 ros2 topic list --no-daemon 2>&1 | head -150
    for t in /clock /scan /odom; do
      section "Tần số $t (10 s)"
      timeout 12 ros2 topic hz "$t" --window 50 2>&1 | tail -3
    done
    section "1 tin /scan (rút gọn)"
    timeout 15 ros2 topic echo --once /scan 2>&1 | grep -E 'frame_id|range_min|range_max' | head
    section "Camera + YOLO"
    timeout 12 ros2 topic hz /oakd/rgb/preview/image_raw --window 20 2>&1 | tail -2
    timeout 12 ros2 topic hz /yolo/image_annotated --window 20 2>&1 | tail -2
    section "/dock_status"
    timeout 15 ros2 topic echo --once /dock_status 2>&1 | head -12
  fi

  section "pip (YOLO)"
  python3 -m pip list 2>/dev/null | grep -i -E '^(numpy|opencv|torch|torchvision|ultralytics) ' 

  section "Log launch gần nhất (~/.ros/log/latest/launch.log, 300 dòng cuối)"
  if [[ -f "${HOME}/.ros/log/latest/launch.log" ]]; then
    tail -n 300 "${HOME}/.ros/log/latest/launch.log"
  else
    echo "không có"
  fi
} > "$OUT" 2>&1

echo "[tb4_lab04] Đã ghi thông tin chẩn đoán vào: $OUT"
echo "[tb4_lab04] Gửi file này (kèm ảnh chụp màn hình Gazebo/RViz nếu có) cho người hỗ trợ."
