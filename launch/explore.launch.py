# Tự khám phá + lập bản đồ: Gazebo + TurtleBot 4 + SLAM Toolbox + Nav2 + auto_explore.py.
#
#   ros2 launch tb4_lab04 explore.launch.py                    # maze, tự lưu maps/map.yaml
#   ros2 launch tb4_lab04 explore.launch.py world:=warehouse
#   ros2 launch tb4_lab04 explore.launch.py explore:=false     # SLAM + Nav2, tự bấm Nav2 Goal
#
# Khác Phần 2 (mapping.launch.py): Nav2 chạy CÙNG SLAM Toolbox (không có map_server/AMCL –
# SLAM vừa vẽ /map vừa phát TF map->odom), và robot được node auto_explore.py điều khiển qua
# action NavigateToPose thay cho teleop.
#
# LƯU Ý: đề bài yêu cầu Phần 2 điều khiển bằng teleop. Dùng launch này cho video nộp bài là
# quyết định của bạn – hãy hỏi giảng viên trước.
#
# Xem "LƯU Ý KỸ THUẬT" trong tb4_lab04_common.py về scope của launch configuration ở Humble.

import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction, IncludeLaunchDescription,
                            LogInfo, TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter, SetParametersFromFile
from launch_ros.parameter_descriptions import ParameterValue

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import tb4_lab04_common as common  # noqa: E402

BOND_TIMEOUT = 15.0
SCAN_DELAY = 0.1


def generate_launch_description():
    abort = common.abort_if_missing(common.PKGS_SIM + common.PKGS_SLAM + common.PKGS_NAV)
    if abort is not None:
        return abort
    pkg_lab = get_package_share_directory(common.PKG)
    pkg_tb4_nav = get_package_share_directory('turtlebot4_navigation')

    arguments = common.sim_arguments('nav.rviz') + [
        DeclareLaunchArgument('explore', default_value='true', choices=['true', 'false'],
                              description='true = tự khám phá; false = chỉ SLAM + Nav2'),
        DeclareLaunchArgument('save_map', default_value='true', choices=['true', 'false'],
                              description='Tự lưu bản đồ khi khám phá xong'),
        DeclareLaunchArgument('map_dir', default_value='',
                              description="Thư mục lưu map ('' = <source>/maps như save_map.sh)"),
        DeclareLaunchArgument('max_duration', default_value='3600.0',
                              description='Thời gian khám phá tối đa (s mô phỏng)'),
        DeclareLaunchArgument('return_home', default_value='true', choices=['true', 'false'],
                              description='Quay về điểm xuất phát trước khi lưu'),
        DeclareLaunchArgument('slam_params_file',
                              default_value=os.path.join(pkg_lab, 'config', 'slam.yaml'),
                              description='File tham số SLAM Toolbox'),
        DeclareLaunchArgument('nav2_params_file',
                              default_value=os.path.join(pkg_lab, 'config', 'nav2_explore.yaml'),
                              description='File tham số Nav2 (bản cho khám phá: nhanh hơn, '
                                          'không xoay đúng hướng ở mỗi goal, inflation 0.35 m)'),
        DeclareLaunchArgument('fast', default_value='true', choices=['true', 'false'],
                              description='true = Create 3 safety_override "full": tới 0.46 m/s '
                                          '(false: 0.306 m/s như mặc định)'),
        DeclareLaunchArgument('slam_delay', default_value='2.0',
                              description='Giây chờ thêm sau khi mô phỏng chạy rồi bật SLAM'),
        DeclareLaunchArgument('nav2_delay', default_value='8.0',
                              description='Giây chờ thêm sau khi mô phỏng chạy rồi bật Nav2'),
        DeclareLaunchArgument('explore_delay', default_value='20.0',
                              description='Giây chờ thêm rồi bật auto_explore.py '
                                          '(node tự chờ tiếp tới khi Nav2 sẵn sàng)'),
    ]

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
                        'sync': 'true',
                        'params': LaunchConfiguration('slam_params_file'),
                    }.items(),
                ),
            ]),
        ],
    )

    nav2 = TimerAction(
        period=LaunchConfiguration('nav2_delay'),
        actions=[
            LogInfo(msg='[tb4_lab04] Khởi động Nav2 (dùng /map của SLAM Toolbox) ...'),
            GroupAction(scoped=True, actions=[
                SetParameter(name='bond_timeout', value=BOND_TIMEOUT),
                # Dock thấp hơn mặt quét LiDAR -> đánh dấu cho costmap (dock_layer trong nav2.yaml).
                Node(package='tb4_lab04', executable='dock_obstacle.py',
                     name='tb4_lab04_dock_obstacle', output='screen'),
                # /scan_nav cho costmap của Nav2 – xem navigation.launch.py (deadlock tf2).
                Node(package='topic_tools', executable='delay', name='scan_delay',
                     output='screen',
                     parameters=[{'input_topic': '/scan', 'output_topic': '/scan_nav',
                                  'delay': SCAN_DELAY, 'use_sim_time': True}]),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(pkg_tb4_nav, 'launch', 'nav2.launch.py')),
                    launch_arguments={
                        'namespace': '',
                        'use_sim_time': 'true',
                        'params_file': LaunchConfiguration('nav2_params_file'),
                    }.items(),
                ),
            ]),
        ],
    )

    explorer = TimerAction(
        period=LaunchConfiguration('explore_delay'),
        condition=IfCondition(LaunchConfiguration('explore')),
        actions=[
            Node(package='tb4_lab04', executable='auto_explore.py',
                 name='tb4_lab04_auto_explore', output='screen',
                 parameters=[{
                     'use_sim_time': True,
                     'save_on_ctrl_c': False,   # Ctrl+C tắt cả SLAM -> không thể lưu; dùng autosave
                     'save_map': ParameterValue(LaunchConfiguration('save_map'), value_type=bool),
                     'map_dir': ParameterValue(LaunchConfiguration('map_dir'), value_type=str),
                     'max_duration': ParameterValue(LaunchConfiguration('max_duration'),
                                                    value_type=float),
                     'return_home': ParameterValue(LaunchConfiguration('return_home'),
                                                   value_type=bool),
                 }]),
        ],
    )

    wait_for_sim = common.wait_for_sim_node()
    start_stack = common.after_sim_ready(wait_for_sim, [slam, nav2, explorer])

    return LaunchDescription(arguments + common.render_env() + [
        common.rviz_node(),
        common.turtlebot4_sim(extra_param_actions=[SetParametersFromFile(
            os.path.join(pkg_lab, 'config', 'create3_fast.yaml'),
            condition=IfCondition(LaunchConfiguration('fast')))]),
        wait_for_sim, start_stack])
