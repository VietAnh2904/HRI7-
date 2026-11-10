# Bài thực hành 04 – Phần 2: xây dựng bản đồ bằng SLAM Toolbox.
#
# Khởi chạy: Gazebo + TurtleBot 4 (launch chính thức) + SLAM Toolbox
# (turtlebot4_navigation/slam.launch.py với config/slam.yaml của repo) + RViz2 (fixed frame map).
# Điều khiển robot bằng teleop ở terminal khác, lưu map bằng `ros2 run tb4_lab04 save_map.sh`.
#
#   ros2 launch tb4_lab04 mapping.launch.py
#   ros2 launch tb4_lab04 mapping.launch.py world:=warehouse sync:=false
#
# SLAM chỉ được bật khi mô phỏng THỰC SỰ chạy (wait_for_sim.py thoát: có /clock và /odom),
# không đoán bằng số giây – trên máy chậm Gazebo có thể mất vài phút mới lên.

import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction, IncludeLaunchDescription,
                            LogInfo, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import tb4_lab04_common as common  # noqa: E402


def generate_launch_description():
    abort = common.abort_if_missing(common.PKGS_SIM + common.PKGS_SLAM)
    if abort is not None:
        return abort
    pkg_lab = get_package_share_directory(common.PKG)
    pkg_tb4_nav = get_package_share_directory('turtlebot4_navigation')

    arguments = common.sim_arguments('nav.rviz') + [
        DeclareLaunchArgument('sync', default_value='true', choices=['true', 'false'],
                              description='true = sync_slam_toolbox_node (xử lý mọi scan, '
                                          'map đẹp hơn); false = async (nhẹ hơn cho máy yếu)'),
        DeclareLaunchArgument('slam_params_file',
                              default_value=os.path.join(pkg_lab, 'config', 'slam.yaml'),
                              description='File tham số SLAM Toolbox'),
        DeclareLaunchArgument('slam_delay', default_value='2.0',
                              description='Số giây chờ thêm sau khi mô phỏng đã chạy '
                                          '(có /clock và /odom) rồi mới bật SLAM'),
    ]

    # Thứ tự quan trọng: TimerAction -> GroupAction(scoped) -> Include (scope nằm TRONG timer).
    slam = TimerAction(
        period=LaunchConfiguration('slam_delay'),
        actions=[
            LogInfo(msg='[tb4_lab04] Khởi động SLAM Toolbox ...'),
            GroupAction(scoped=True, actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(pkg_tb4_nav, 'launch', 'slam.launch.py')),
                    launch_arguments={
                        'namespace': '',
                        'use_sim_time': 'true',
                        'sync': LaunchConfiguration('sync'),
                        'params': LaunchConfiguration('slam_params_file'),
                    }.items(),
                ),
            ]),
        ],
    )

    wait_for_sim = common.wait_for_sim_node()
    start_slam = common.after_sim_ready(wait_for_sim, [slam])

    return LaunchDescription(
        arguments + common.render_env()
        + [common.rviz_node(), common.turtlebot4_sim(), wait_for_sim, start_slam])
