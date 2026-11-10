# Chạy TOÀN BỘ bài trong một lệnh: Gazebo + vật thể + AMCL + Nav2 + RViz2 + YOLO + node điều hướng tự động.
#
#   ros2 launch tb4_lab04 demo.launch.py
#   ros2 launch tb4_lab04 demo.launch.py run_navigator:=false      # không tự đi – chạy navigator ở terminal khác
#   ros2 launch tb4_lab04 demo.launch.py yolo_model:=yolov8n.pt software_render:=true
#
# Quay video thì nên dùng 3 terminal (README mục 5) để người xem thấy rõ từng bước; file này tiện để chạy thử.
# Mọi tham số của navigation.launch.py và yolo.launch.py đều dùng được ở đây.
#   run_navigator    true   chạy scripts/waypoint_navigator.py (đi Goal 1 -> 2 -> 3)
#   navigator_delay  25.0   giây chờ trước khi chạy navigator (navigator vẫn tự chờ Nav2 active)
#   goals_file       ''     file goal (trống = config/goals.yaml)
#   run_yolo         true   chạy yolo_detector

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PKG = 'tb4_lab04'


def _navigator(context, *args, **kwargs):
    params = {}
    goals = LaunchConfiguration('goals_file').perform(context).strip()
    if goals:
        params['goals_file'] = os.path.abspath(os.path.expanduser(goals))
    return [Node(package=PKG, executable='waypoint_navigator.py', name='tb4_waypoint_navigator',
                 output='screen', emulate_tty=True, parameters=[params])]


def generate_launch_description():
    pkg = get_package_share_directory(PKG)
    return LaunchDescription([
        DeclareLaunchArgument('run_navigator', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('navigator_delay', default_value='25.0'),
        DeclareLaunchArgument('goals_file', default_value=''),
        DeclareLaunchArgument('run_yolo', default_value='true', choices=['true', 'false']),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'navigation.launch.py'))),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'yolo.launch.py')),
            condition=IfCondition(LaunchConfiguration('run_yolo'))),
        TimerAction(period=LaunchConfiguration('navigator_delay'),
                    condition=IfCondition(LaunchConfiguration('run_navigator')),
                    actions=[OpaqueFunction(function=_navigator)]),
    ])
