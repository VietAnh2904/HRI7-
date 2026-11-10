# Phần 4 – node nhận diện vật thể YOLO từ camera TurtleBot 4 (scripts/yolo_detector.py).
#
#   ros2 launch tb4_lab04 yolo.launch.py
#   ros2 launch tb4_lab04 yolo.launch.py yolo_model:=yolov8n.pt conf:=0.3
#   ros2 launch tb4_lab04 yolo.launch.py yolo_model:=yolov8s-worldv2.pt     # YOLO-World
#
# Tham số đầy đủ: config/yolo.yaml. Tham số launch (để trống = giữ giá trị trong file):
#   yolo_model, conf, device, image_topic, show_window, params_file.
# Lưu ý: tên "yolo_model" (không phải "model") vì launch TurtleBot 4 đã dùng "model" (standard/lite) và
# ở ROS 2 Humble các file launch include nhau DÙNG CHUNG tham số.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PKG = 'tb4_lab04'
OVERRIDES = {  # tham số launch -> (tham số node, kiểu)
    'yolo_model': ('model', str),
    'conf': ('conf', float),
    'device': ('device', str),
    'image_topic': ('image_topic', str),
    'show_window': ('show_window', lambda v: v.lower() in ('1', 'true', 'yes')),
}


def _setup(context, *args, **kwargs):
    params = [LaunchConfiguration('yolo_params_file').perform(context)]
    over = {}
    for arg, (name, conv) in OVERRIDES.items():
        val = LaunchConfiguration(arg).perform(context).strip()
        if val:
            over[name] = conv(val)
    if over:
        params.append(over)
    return [Node(package=PKG, executable='yolo_detector.py', name='yolo_detector', output='screen',
                 emulate_tty=True, parameters=params)]


def generate_launch_description():
    pkg = get_package_share_directory(PKG)
    return LaunchDescription([
        DeclareLaunchArgument('yolo_params_file', default_value=os.path.join(pkg, 'config', 'yolo.yaml'),
                              description='File tham số của yolo_detector'),
        DeclareLaunchArgument('yolo_model', default_value='',
                              description='Model YOLO (trống = theo file tham số, mặc định yolo11n.pt)'),
        DeclareLaunchArgument('conf', default_value='', description='Ngưỡng độ tự tin (0..1)'),
        DeclareLaunchArgument('device', default_value='', description='cpu | 0 (GPU) | trống = tự chọn'),
        DeclareLaunchArgument('image_topic', default_value='', description='Topic ảnh RGB'),
        DeclareLaunchArgument('show_window', default_value='',
                              description='true/false – cửa sổ OpenCV'),
        OpaqueFunction(function=_setup),
    ])
