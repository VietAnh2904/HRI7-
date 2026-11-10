#!/usr/bin/env bash
# Bài thực hành 04 – Phần 2: lưu bản đồ đang được SLAM Toolbox xây dựng.
#
#   ros2 run tb4_lab04 save_map.sh               # lưu vào <source>/maps/map.{yaml,pgm}
#   ros2 run tb4_lab04 save_map.sh /thu/muc/khac # lưu vào thư mục chỉ định
#
# Chạy khi mapping.launch.py VẪN ĐANG CHẠY (SLAM phải còn publish /map).
# Bản đồ cũ (nếu có) được chuyển vào maps/backup_<thời_gian>/ trước khi ghi đè.
#
# Vì sao có "--free 0.15"? map_saver_cli của Nav2 Humble mặc định ghi free_thresh = 0.25,
# trong khi ô "chưa biết" được ghi bằng màu xám 205 (xác suất ~0.196 < 0.25). Khi load lại,
# map_server sẽ coi vùng CHƯA KHẢO SÁT là vùng TRỐNG -> Nav2 có thể lập đường đi xuyên vùng
# chưa biết. Dùng 0.15 thì 205 được đọc đúng là "chưa biết".

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)"

err() { echo -e "\e[31m[LỖI]\e[0m $*" >&2; }
warn() { echo -e "\e[33m[CẢNH BÁO]\e[0m $*" >&2; }
info() { echo -e "\e[32m[tb4_lab04]\e[0m $*"; }

if ! command -v ros2 >/dev/null 2>&1; then
  err "Không thấy lệnh ros2. Chạy: source /opt/ros/humble/setup.bash && source ~/workspaces/tb4_gazebo/install/setup.bash"
  exit 1
fi

# ---- Xác định thư mục lưu ----
if [[ $# -ge 1 && -n "$1" ]]; then
  OUT_DIR="$1"
else
  OUT_DIR=""
  if PREFIX="$(ros2 pkg prefix tb4_lab04 2>/dev/null)"; then
    MARKER="$PREFIX/share/tb4_lab04/source_dir.txt"
    if [[ -f "$MARKER" ]]; then
      SRC="$(head -n 1 "$MARKER")"
      if [[ -d "$SRC" && -w "$SRC" ]]; then
        OUT_DIR="$SRC/maps"
      fi
    fi
  fi
  if [[ -z "$OUT_DIR" ]]; then
    OUT_DIR="$PWD/maps"
    info "Không xác định được thư mục source của package -> lưu vào $OUT_DIR"
  fi
fi
mkdir -p "$OUT_DIR"
OUT_DIR="$(cd "$OUT_DIR" && pwd)"

# ---- Kiểm tra có node đang publish /map ----
# --no-daemon: KHÔNG hỏi qua ROS 2 daemon. Daemon được khởi động bởi terminal ĐẦU TIÊN gọi lệnh
# ros2 và giữ môi trường của terminal đó (ROS_LOCALHOST_ONLY, ROS_DOMAIN_ID...). Nếu khác môi trường
# hiện tại, daemon có thể "không thấy" SLAM Toolbox dù nó vẫn đang publish /map (đã gặp thực tế).
# Chỉ CẢNH BÁO: map_saver_cli bên dưới tự chờ /map và sẽ báo lỗi rõ ràng nếu thật sự không có.
PUBS="$(timeout 30 ros2 topic info --no-daemon --spin-time 5 /map 2>/dev/null \
        | awk -F': ' '/Publisher count/ {print $2}' || true)"
if [[ -z "$PUBS" || "$PUBS" == "0" ]]; then
  warn "Chưa thấy node nào publish /map (SLAM Toolbox). Vẫn thử lưu (chờ tối đa 20 s) ..."
  warn "Nếu thất bại: để mapping.launch.py/explore.launch.py CHẠY rồi lưu; kiểm tra mọi terminal cùng"
  warn "ROS_LOCALHOST_ONLY và ROS_DOMAIN_ID (echo \$ROS_LOCALHOST_ONLY \$ROS_DOMAIN_ID)."
fi

# ---- Lưu vào thư mục tạm trước; chỉ thay bản cũ khi bản mới hợp lệ ----
TMP_DIR="$(mktemp -d "$OUT_DIR/.saving_XXXXXX")"
trap 'rm -rf "$TMP_DIR"' EXIT

info "Đang lưu bản đồ ..."
if ! ros2 run nav2_map_server map_saver_cli \
      -f "$TMP_DIR/map" --occ 0.65 --free 0.15 --fmt pgm --mode trinary \
      --ros-args -p save_map_timeout:=20.0 -p map_subscribe_transient_local:=true; then
  err "map_saver_cli thất bại (xem log phía trên). Bản đồ cũ (nếu có) vẫn giữ nguyên."
  exit 1
fi
for f in map.yaml map.pgm; do
  if [[ ! -s "$TMP_DIR/$f" ]]; then
    err "map_saver_cli không tạo ra $f. Bản đồ cũ (nếu có) vẫn giữ nguyên."
    exit 1
  fi
done

# ---- Sao lưu bản đồ cũ rồi đưa bản mới vào chỗ ----
if [[ -f "$OUT_DIR/map.yaml" || -f "$OUT_DIR/map.pgm" ]]; then
  BK="$OUT_DIR/backup_$(date +%Y%m%d_%H%M%S)"
  mkdir -p "$BK"
  for f in map.yaml map.pgm; do
    if [[ -f "$OUT_DIR/$f" ]]; then mv "$OUT_DIR/$f" "$BK/"; fi
  done
  info "Đã chuyển bản đồ cũ vào $BK"
fi
mv "$TMP_DIR/map.yaml" "$TMP_DIR/map.pgm" "$OUT_DIR/"

info "Đã lưu xong:"
ls -l "$OUT_DIR/map.yaml" "$OUT_DIR/map.pgm"
echo
python3 "$SCRIPT_DIR/check_map.py" "$OUT_DIR/map.yaml" || true
echo
info "Bước tiếp theo (Phần 3): dừng mapping (Ctrl+C), chạy  ros2 run tb4_lab04 kill_sim.sh"
info "rồi:  ros2 launch tb4_lab04 navigation.launch.py use_nav2:=false"
