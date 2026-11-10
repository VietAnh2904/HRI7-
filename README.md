#!/bin/bash
set -e

echo "========================================"
echo "TURTLEBOT 4 VOICE NAVIGATION - AUTO RUN"
echo "========================================"

# 1. Source ROS 2 Humble
source /opt/ros/humble/setup.bash

# 2. Tạo workspace và clone (nếu chưa có)
WS="$HOME/workspaces/tb4_gazebo"
SRC="$WS/src"
PKG="$SRC/tb4_lab04"

mkdir -p "$SRC"
cd "$SRC"

if [ ! -d "$PKG" ]; then
    echo "[+] Cloning repository..."
    git clone https://github.com/VietAnh2904/HRI7-.git tb4_lab04
fi

cd "$WS"

# 3. Build package
echo "[+] Building with colcon..."
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select tb4_lab04
source install/setup.bash
export ROS_LOCALHOST_ONLY=1

# 4. Cài dependencies giọng nói
echo "[+] Installing voice dependencies..."
cd "$PKG"
bash scripts/install_voice_deps.sh

# 5. Chạy phân vùng bản đồ (1 lần)
echo "[+] Running zone segmentation..."
cd "$WS"
source install/setup.bash
export ROS_LOCALHOST_ONLY=1
ros2 run tb4_lab04 zone_segmentation.py

echo ""
echo "========================================"
echo "SETUP HOAN TAT"
echo "========================================"
echo ""
echo "MOI BAO MO 3 TERMINAL MOI VA CHAY CAC LENH DUOI DAY:"
echo ""
echo "---------------- Terminal 1 ----------------"
echo "# Gazebo + Nav2 + RViz"
echo "source /opt/ros/humble/setup.bash"
echo "source $WS/install/setup.bash"
echo "export ROS_LOCALHOST_ONLY=1"
echo "cd $WS"
echo "ros2 run tb4_lab04 kill_sim.sh"
echo "ros2 launch tb4_lab04 navigation.launch.py rviz_config:=\$(ros2 pkg prefix tb4_lab04)/share/tb4_lab04/rviz/voice.rviz"
echo ""
echo "=> Choi den khi in 2 lan 'Managed nodes are active'"
echo ""
echo "---------------- Terminal 2 ----------------"
echo "# Giong noi + Dieu huong"
echo "source /opt/ros/humble/setup.bash"
echo "source $WS/install/setup.bash"
echo "export ROS_LOCALHOST_ONLY=1"
echo "ros2 launch tb4_lab04 voice.launch.py"
echo ""
echo "=> Choi den khi in 'San sang nhan lenh' va 'Dang nghe micro ...'"
echo ""
echo "---------------- Terminal 3 ----------------"
echo "# Xem phan hoi robot (tuychon)"
echo "source /opt/ros/humble/setup.bash"
echo "source $WS/install/setup.bash"
echo "export ROS_LOCALHOST_ONLY=1"
echo "ros2 topic echo /voice/feedback"
echo ""
echo "---------------- LENH MAU (noi vao micro) ----------------"
echo "1. 'Di den khu vuc G.'"
echo "2. 'Di den khu vuc Z.'  (khu vuc khong ton tai -> robot dung yen)"
echo "3. 'Di chuyen tu sanh chinh den khu vuc C.'"
echo "4. 'Di den phong thuc nghiem.'"
echo ""
echo "---------------- KHONG CO MIC ----------------"
echo "Go lenh thay cho noi:"
echo "ros2 run tb4_lab04 voice_keyboard.py \"Di den khu vuc G\""
echo ""
echo "========================================"
