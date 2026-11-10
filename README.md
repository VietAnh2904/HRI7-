# TurtleBot 4 Voice Navigation

Dự án điều hướng TurtleBot 4 bằng giọng nói trên ROS 2 Humble và Gazebo Simulation.

Hệ thống hỗ trợ phân vùng bản đồ, nhận diện câu lệnh giọng nói và điều hướng robot đến các khu vực được chỉ định.

## 1. Môi trường

- Ubuntu 22.04
- ROS 2 Humble
- TurtleBot 4 Simulation
- Gazebo
- Nav2
- RViz2

## 2. Cấu trúc dự án

```text
tb4_lab04/
├── config/       # Các file cấu hình
├── launch/       # Launch files
├── maps/         # Bản đồ và dữ liệu phân vùng
├── rviz/         # Cấu hình RViz
├── scripts/      # Các chương trình Python và Bash
├── CMakeLists.txt
├── package.xml
└── README.md
```

## 3. Cài đặt

Tạo workspace:

```bash
mkdir -p ~/workspaces/tb4_gazebo/src
cd ~/workspaces/tb4_gazebo/src
```

Clone repository:

```bash
git clone https://github.com/VietAnh2904/HRI7-.git tb4_lab04
```

Build package:

```bash
cd ~/workspaces/tb4_gazebo

source /opt/ros/humble/setup.bash

colcon build --symlink-install --packages-select tb4_lab04

source install/setup.bash
```

Cài đặt các thư viện phục vụ nhận diện giọng nói:

```bash
cd ~/workspaces/tb4_gazebo/src/tb4_lab04

bash scripts/install_voice_deps.sh
```

## 4. Phân vùng bản đồ

Sau khi build và cài đặt dependencies, chạy:

```bash
source /opt/ros/humble/setup.bash
source ~/workspaces/tb4_gazebo/install/setup.bash

export ROS_LOCALHOST_ONLY=1

ros2 run tb4_lab04 zone_segmentation.py
```

## 5. Khởi chạy hệ thống

Mở 3 Terminal riêng biệt.

### Terminal 1: Gazebo, Nav2 và RViz

```bash
source /opt/ros/humble/setup.bash
source ~/workspaces/tb4_gazebo/install/setup.bash

export ROS_LOCALHOST_ONLY=1

cd ~/workspaces/tb4_gazebo

ros2 run tb4_lab04 kill_sim.sh

ros2 launch tb4_lab04 navigation.launch.py rviz_config:=$(ros2 pkg prefix tb4_lab04)/share/tb4_lab04/rviz/voice.rviz
```

Chờ Nav2 khởi động hoàn tất.

### Terminal 2: Nhận diện giọng nói và điều hướng

```bash
source /opt/ros/humble/setup.bash
source ~/workspaces/tb4_gazebo/install/setup.bash

export ROS_LOCALHOST_ONLY=1

ros2 launch tb4_lab04 voice.launch.py
```

Chờ hệ thống hiển thị:

- San sang nhan lenh
- Dang nghe micro ...

### Terminal 3: Theo dõi phản hồi

```bash
source /opt/ros/humble/setup.bash
source ~/workspaces/tb4_gazebo/install/setup.bash

export ROS_LOCALHOST_ONLY=1

ros2 topic echo /voice/feedback
```

## 6. Các câu lệnh điều khiển mẫu

Nói trực tiếp vào microphone:

| STT | Câu lệnh |
|---|---|
| 1 | Đi đến khu vực G |
| 2 | Đi đến khu vực Z |
| 3 | Di chuyển từ sảnh chính đến khu vực C |
| 4 | Đi đến phòng thí nghiệm |

Khu vực Z là ví dụ về vị trí không tồn tại trong bản đồ, dùng để kiểm tra khả năng xử lý địa điểm không hợp lệ.

## 7. Điều khiển không cần microphone

Có thể nhập câu lệnh trực tiếp bằng bàn phím:

```bash
ros2 run tb4_lab04 voice_keyboard.py "Di den khu vuc G"
```

## 8. Khởi chạy tự động

Repository có thể sử dụng script `setup_voice_navigation.sh` để tự động thực hiện các bước:

1. Source ROS 2 Humble.
2. Tạo workspace và clone repository.
3. Build package bằng colcon.
4. Cài đặt dependencies giọng nói.
5. Chạy chương trình phân vùng bản đồ.

Chạy bằng lệnh:

```bash
bash setup_voice_navigation.sh
```
