# Bài 6 – Phần 2 + 3: node nhận diện cử chỉ (webcam + MediaPipe) và node điều khiển hành vi TurtleBot 4.
#
#   ros2 launch tb4_lab04 gesture.launch.py                       # mô phỏng đã chạy ở terminal khác
#   ros2 launch tb4_lab04 gesture.launch.py camera_source:=1      # webcam USB
#   ros2 launch tb4_lab04 gesture.launch.py wave_requires_confirm:=false   # vẫy tay -> chào ngay
#   ros2 launch tb4_lab04 gesture.launch.py run_behavior:=false   # chỉ thử nhận diện (không có robot)
#
# Tham số đầy đủ: config/gesture.yaml. Tham số launch để trống = giữ giá trị trong file.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PKG = 'tb4_lab04'


def _bool(v):
    return v.strip().lower() in ('1', 'true', 'yes', 'on')


# tham số launch -> (node, tên tham số, hàm đổi kiểu)
OVERRIDES = {
    'camera_source': ('rec', 'camera_source', str),
    'show_window': ('rec', 'show_window', _bool),
    'mirror': ('rec', 'mirror', _bool),
    'window_position': ('rec', 'window_position', str),
    'wave_requires_confirm': ('beh', 'wave_requires_confirm', _bool),
    'auto_undock': ('beh', 'auto_undock', _bool),
    'setup_forward': ('beh', 'setup_forward', float),
}


def _setup(context, *args, **kwargs):
    params_file = LaunchConfiguration('gesture_params_file').perform(context)
    over = {'rec': {}, 'beh': {}}
    for arg, (who, name, conv) in OVERRIDES.items():
        val = LaunchConfiguration(arg).perform(context).strip()
        if val:
            over[who][name] = conv(val)
    nodes = []
    if _bool(LaunchConfiguration('run_recognition').perform(context)):
        nodes.append(Node(package=PKG, executable='gesture_recognition_node.py',
                          name='gesture_recognition_node', output='screen', emulate_tty=True,
                          parameters=[params_file, over['rec']]))
    if _bool(LaunchConfiguration('run_behavior').perform(context)):
        nodes.append(Node(package=PKG, executable='robot_behavior_node.py',
                          name='robot_behavior_node', output='screen', emulate_tty=True,
                          parameters=[params_file, over['beh']]))
    return nodes


def generate_launch_description():
    pkg = get_package_share_directory(PKG)
    return LaunchDescription([
        DeclareLaunchArgument('gesture_params_file',
                              default_value=os.path.join(pkg, 'config', 'gesture.yaml'),
                              description='File tham số của 2 node cử chỉ'),
        DeclareLaunchArgument('run_recognition', default_value='true', choices=['true', 'false'],
                              description='Chạy gesture_recognition_node (webcam + MediaPipe)'),
        DeclareLaunchArgument('run_behavior', default_value='true', choices=['true', 'false'],
                              description='Chạy robot_behavior_node (/cmd_vel)'),
        DeclareLaunchArgument('camera_source', default_value='',
                              description='0 = webcam laptop, 1 = webcam USB, /dev/videoN, file video, thư mục ảnh'),
        DeclareLaunchArgument('show_window', default_value='', description='true/false – cửa sổ webcam'),
        DeclareLaunchArgument('mirror', default_value='', description='true/false – lật gương ảnh webcam'),
        DeclareLaunchArgument('window_position', default_value='',
                              description='"x,y" – vị trí cửa sổ webcam trên màn hình (vd 1000,0)'),
        DeclareLaunchArgument('wave_requires_confirm', default_value='',
                              description='true: vẫy tay chỉ CHỌN chào (cần ngón cái); false: chào ngay'),
        DeclareLaunchArgument('auto_undock', default_value='',
                              description='true/false – tự rời dock + ra giữa sân khi khởi động'),
        DeclareLaunchArgument('setup_forward', default_value='',
                              description='m – quãng chạy thẳng ra giữa sân sau khi undock'),
        OpaqueFunction(function=_setup),
    ])
