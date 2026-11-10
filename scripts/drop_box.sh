#!/usr/bin/env bash
# Bài 7 – thả một khối hộp (vật cản KHÔNG có trên bản đồ) vào Gazebo để thử Nav2 tránh vật cản.
#   ros2 run tb4_lab04 drop_box.sh 2.2 2.0            # hộp 0.45 m tại (2.2, 2.0) – trên đường sảnh -> khu vực C
#   ros2 run tb4_lab04 drop_box.sh 2.2 2.0 hop2 0.6   # tên khác, cạnh 0.6 m
#   ros2 run tb4_lab04 drop_box.sh --remove hop1      # xoá hộp
# Đặt hộp lên đúng đường màu cam (Voice Planned Route) trên RViz2, cách điểm đích >= 1 m.
set -u
WORLD="${TB4_WORLD:-maze}"
if [ "${1:-}" = "--remove" ]; then
  NAME="${2:-hop1}"
  exec ign service -s "/world/${WORLD}/remove" --reqtype ignition.msgs.Entity --reptype ignition.msgs.Boolean \
    --timeout 5000 --req "name: \"${NAME}\" type: MODEL"
fi
X="${1:?Cách dùng: drop_box.sh X Y [tên] [cạnh_m]}"
Y="${2:?Cách dùng: drop_box.sh X Y [tên] [cạnh_m]}"
NAME="${3:-hop1}"
S="${4:-0.45}"
SDF="<?xml version=\"1.0\"?><sdf version=\"1.8\"><model name=\"${NAME}\"><static>true</static><link name=\"link\">\
<collision name=\"c\"><geometry><box><size>${S} ${S} 0.6</size></box></geometry></collision>\
<visual name=\"v\"><geometry><box><size>${S} ${S} 0.6</size></box></geometry>\
<material><ambient>0.8 0.1 0.1 1</ambient><diffuse>0.8 0.1 0.1 1</diffuse></material></visual></link></model></sdf>"
exec ros2 run ros_gz_sim create -world "${WORLD}" -name "${NAME}" -x "${X}" -y "${Y}" -z 0.3 -string "${SDF}"
