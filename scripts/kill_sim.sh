#!/usr/bin/env bash
# Bài thực hành 04 – dọn sạch Gazebo/ROS còn sót lại giữa các phần.
#
#   ros2 run tb4_lab04 kill_sim.sh
#
# Vì sao cần? Sau Ctrl+C, Ignition Gazebo (tiến trình "ign gazebo" chạy bằng ruby) và vài
# node ROS đôi khi KHÔNG tắt hẳn. Nếu mở launch mới khi còn sim cũ: 2 server Gazebo cùng
# publish /clock, robot "nhảy", TF lỗi, AMCL không hội tụ -> lỗi rất khó hiểu.
#
# Script chỉ dừng tiến trình của user hiện tại thuộc các nhóm sau:
#   * Ignition Gazebo (ign gazebo / gz sim)
#   * mọi node ROS 2 (có tham số --ros-args) và mọi lệnh `ros2 run` (teleop, check_sim...) – trừ chính
#     terminal này và các shell (bash/sh...)
# Gửi SIGINT, chờ 5 s, còn sót thì SIGKILL. Sau đó tắt ros2 daemon để xoá cache discovery.

set -uo pipefail

ME=$$
PARENT=$PPID

collect() {
  # pid và cmdline của tiến trình thuộc user hiện tại khớp mẫu
  # Bỏ qua shell (bash/sh/zsh/dash...): một shell chạy kiểu `bash -c "...ros2 launch..."` có dòng lệnh
  # khớp mẫu nhưng KHÔNG phải tiến trình ROS – giết nó sẽ cắt ngang lệnh của chính người dùng.
  pgrep -u "$(id -u)" -f -a -- "$1" 2>/dev/null \
    | awk -v me="$ME" -v pp="$PARENT" '$1!=me && $1!=pp' \
    | while read -r pid rest; do
        comm="$(cat "/proc/$pid/comm" 2>/dev/null || true)"
        case "$comm" in bash|sh|dash|zsh|fish|ksh|tcsh|csh|"") continue ;; esac
        echo "$pid $rest"
      done
}

list_targets() {
  {
    collect 'ign gazebo'
    collect 'gz sim'
    collect '--ros-args'
    collect 'ros2 launch'
    # Cả node chạy bằng `ros2 run` (teleop, check_sim, compare_pose...): còn sót sẽ tiếp tục publish
    # /cmd_vel hoặc giữ file shared-memory sang phiên mới (đã gặp khi kiểm thử).
    collect 'ros2 run'
    collect '/lib/tb4_lab04/'
    collect 'teleop_twist_keyboard'
  } | grep -v 'kill_sim.sh' | sort -u -k1,1n
}

TARGETS="$(list_targets)"
if [[ -z "$TARGETS" ]]; then
  echo "[tb4_lab04] Không còn tiến trình Gazebo/ROS nào đang chạy."
else
  echo "[tb4_lab04] Sẽ dừng các tiến trình sau:"
  echo "$TARGETS" | cut -c1-150
  PIDS="$(echo "$TARGETS" | awk '{print $1}')"
  # shellcheck disable=SC2086
  kill -INT $PIDS 2>/dev/null || true
  for _ in 1 2 3 4 5; do
    sleep 1
    LEFT=""
    for p in $PIDS; do
      if kill -0 "$p" 2>/dev/null; then LEFT="$LEFT $p"; fi
    done
    [[ -z "$LEFT" ]] && break
  done
  if [[ -n "${LEFT:-}" ]]; then
    echo "[tb4_lab04] Còn sót:$LEFT -> SIGKILL"
    # shellcheck disable=SC2086
    kill -KILL $LEFT 2>/dev/null || true
  fi
fi

if command -v ros2 >/dev/null 2>&1; then
  ros2 daemon stop >/dev/null 2>&1 || true
fi

# File shared-memory của FastDDS (/dev/shm/fastrtps_*) do tiến trình bị SIGKILL để lại làm lần chạy
# sau lỗi discovery chập chờn: "robot_state_publisher service not available", node "biến mất" khỏi
# `ros2 node list`, log "RTPS_TRANSPORT_SHM Error ... open_and_lock_file failed". Chỉ xoá khi user
# này KHÔNG còn tiến trình ROS 2 nào (để không phá ứng dụng ROS khác đang chạy).
sleep 1
if [[ -z "$( { collect '--ros-args'; collect 'ros2 launch'; collect 'ros2 run'; collect '_ros2_daemon'; } | grep -v 'kill_sim.sh')" ]]; then
  N=$(find /dev/shm -maxdepth 1 -user "$(id -u)" \( -name 'fastrtps_*' -o -name 'sem.fastrtps_*' \
        -o -name 'fast_datasharing_*' -o -name 'sem.fast_datasharing_*' \) 2>/dev/null | wc -l)
  if (( N > 0 )); then
    find /dev/shm -maxdepth 1 -user "$(id -u)" \( -name 'fastrtps_*' -o -name 'sem.fastrtps_*' \
        -o -name 'fast_datasharing_*' -o -name 'sem.fast_datasharing_*' \) -delete 2>/dev/null || true
    echo "[tb4_lab04] Đã xoá $N file shared-memory cũ của FastDDS trong /dev/shm."
  fi
else
  echo "[tb4_lab04] Còn tiến trình ROS 2 khác đang chạy -> không xoá /dev/shm/fastrtps_*."
fi
echo "[tb4_lab04] Đã dọn xong. Có thể khởi chạy launch mới."
