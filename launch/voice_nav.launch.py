# Bài 7 – TẤT CẢ trong một lệnh: Gazebo + TurtleBot 4 + AMCL + Nav2 + RViz2 (khu vực, đường dự kiến)
#         + nhận dạng giọng nói + điều hướng theo câu lệnh.
#
#   ros2 launch tb4_lab04 voice_nav.launch.py
#   ros2 launch tb4_lab04 voice_nav.launch.py asr_engine:=whisper speak:=true
#   ros2 launch tb4_lab04 voice_nav.launch.py use_asr:=false        # gõ lệnh: ros2 run tb4_lab04 voice_keyboard.py
#   ros2 launch tb4_lab04 voice_nav.launch.py spawn_objects:=false software_render:=true
#
# Mọi tham số của navigation.launch.py và voice.launch.py đều dùng được. Thêm:
#   voice_delay  20.0  giây chờ (sau khi launch) rồi mới bật 2 node giọng nói – để log khởi động Gazebo/Nav2
#                      không lấp mất dòng "NGHE ĐƯỢC ...". voice_navigator tự chờ Nav2 khi nhận lệnh đầu.

import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import tb4_lab04_common as common  # noqa: E402

PKG = 'tb4_lab04'


def generate_launch_description():
    abort = common.abort_if_missing(common.PKGS_SIM + common.PKGS_NAV)
    if abort is not None:
        return abort
    pkg = get_package_share_directory(PKG)
    sys.path.insert(0, os.path.join(pkg, 'launch'))
    import importlib.util
    spec = importlib.util.spec_from_file_location('tb4_voice_launch', os.path.join(pkg, 'launch', 'voice.launch.py'))
    voice = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(voice)
    return LaunchDescription(voice.voice_arguments() + [
        DeclareLaunchArgument('voice_delay', default_value='20.0',
                              description='Giây chờ trước khi bật node giọng nói'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'navigation.launch.py')),
            launch_arguments={'rviz_config': os.path.join(pkg, 'rviz', 'voice.rviz')}.items()),
        TimerAction(period=LaunchConfiguration('voice_delay'),
                    actions=[OpaqueFunction(function=voice.voice_actions)]),
    ])
