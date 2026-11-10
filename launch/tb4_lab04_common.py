# Phần dùng chung cho các launch file của tb4_lab04 (không phải launch file – đừng chạy trực tiếp).
#
# Gom vào một chỗ những thứ trước đây bị chép lặp ở 4 launch file:
#   * kiểm tra trước các gói apt cần thiết  -> báo lỗi RÕ RÀNG kèm lệnh `sudo apt install ...`
#     rồi dừng, thay vì traceback "package '...' not found" khó hiểu;
#   * các tham số chung (world, model, vị trí spawn, RViz, cliff_reflex, software_render);
#   * khởi chạy Gazebo + TurtleBot 4 bằng launch CHÍNH THỨC turtlebot4_ignition_bringup;
#   * node RViz2 và node wait_for_sim.py.
#
# LƯU Ý KỸ THUẬT (đừng "dọn dẹp" mấy chỗ này):
#   * Trong ROS 2 Humble, IncludeLaunchDescription KHÔNG tạo scope riêng: mọi tham số truyền vào
#     include (rviz, slam, nav2, namespace, ...) bị ghi vào context chung. Vì vậy tham số của repo
#     được đặt tên khác hẳn (use_rviz, rviz_config, slam_params_file, ...) để không bị launch của
#     TurtleBot 4 ghi đè.
#   * KHÔNG bọc include của TurtleBot 4 trong GroupAction(scoped=True): bên trong có event handler
#     (OnProcessExit -> spawner diffdrive_controller) chạy SAU khi scope đã đóng và cần đọc launch
#     configuration 'namespace'. Bọc scope -> lỗi "launch configuration 'namespace' does not exist",
#     robot không có diffdrive_controller -> không có /odom.
#   * Các include SLAM / localization / Nav2 thì ngược lại: bọc GroupAction(scoped=True) đặt BÊN
#     TRONG TimerAction (scope phải mở lúc timer kích hoạt).

import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, LogInfo, RegisterEventHandler,
                            SetEnvironmentVariable, Shutdown)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParametersFromFile

PKG = 'tb4_lab04'
WORLDS = ['maze', 'depot', 'warehouse', 'gesture_arena']  # gesture_arena: worlds/ của repo (Bài 6)

# Gói cần cho từng bước (tên gói ROS; tên apt = ros-humble-<tên, '_' -> '-'>).
PKGS_SIM = ['turtlebot4_ignition_bringup', 'turtlebot4_description', 'turtlebot4_navigation',
            'turtlebot4_viz', 'irobot_create_common_bringup', 'irobot_create_ignition_bringup',
            'irobot_create_nodes', 'ros_ign_gazebo', 'ros_gz_bridge', 'rviz2']
PKGS_SLAM = ['slam_toolbox']
PKGS_NAV = ['nav2_bringup', 'nav2_amcl', 'nav2_map_server', 'nav2_rviz_plugins', 'topic_tools']


def missing_packages_message(names):
    """Thông báo lỗi (kèm lệnh apt) nếu thiếu gói, hoặc None nếu đủ."""
    missing = []
    for name in names:
        try:
            get_package_share_directory(name)
        except PackageNotFoundError:
            missing.append(name)
    if not missing:
        return None
    apt = ' '.join('ros-humble-' + m.replace('_', '-') for m in missing)
    return ('\n\n[tb4_lab04] THIẾU GÓI ROS: {}\n'
            '  Cài bằng:  sudo apt update && sudo apt install -y {}\n'
            '  Rồi mở terminal mới (hoặc source /opt/ros/humble/setup.bash) và chạy lại.\n'
            '  Kiểm tra toàn bộ môi trường: ros2 run tb4_lab04 doctor.sh\n'.format(
                ', '.join(missing), apt))


def abort_if_missing(names):
    """None nếu đủ gói; ngược lại một LaunchDescription chỉ in lỗi rồi dừng (không mở Gazebo,
    không traceback khó đọc)."""
    msg = missing_packages_message(names)
    if msg is None:
        return None
    return LaunchDescription([LogInfo(msg=msg), Shutdown(reason='tb4_lab04: thiếu gói ROS')])


def sim_arguments(rviz_file, world_note='World Gazebo có sẵn của TurtleBot 4', default_world='maze',
                  default_rviz='true'):
    """Tham số chung của mọi launch có mô phỏng."""
    pkg_lab = get_package_share_directory(PKG)
    return [
        DeclareLaunchArgument('world', default_value=default_world, choices=WORLDS,
                              description=world_note),
        DeclareLaunchArgument('model', default_value='standard', choices=['standard', 'lite'],
                              description='Mẫu TurtleBot 4'),
        DeclareLaunchArgument('x', default_value='0.0', description='Vị trí spawn x (m)'),
        DeclareLaunchArgument('y', default_value='0.0', description='Vị trí spawn y (m)'),
        DeclareLaunchArgument('z', default_value='0.0', description='Vị trí spawn z (m)'),
        DeclareLaunchArgument('yaw', default_value='0.0', description='Hướng spawn (rad)'),
        DeclareLaunchArgument('use_rviz', default_value=default_rviz, choices=['true', 'false'],
                              description='Mở RViz2'),
        DeclareLaunchArgument('rviz_config',
                              default_value=os.path.join(pkg_lab, 'rviz', rviz_file),
                              description='File cấu hình RViz2'),
        DeclareLaunchArgument(
            'cliff_reflex', default_value='false', choices=['true', 'false'],
            description='Phản xạ cliff (chống rơi) của Create 3. Mặc định TẮT: world phẳng, còn cảm '
                        'biến cliff (gpu_lidar) đo sai khi render đồ hoạ lỗi sẽ huỷ undock và làm '
                        'robot bỏ qua /cmd_vel. Phản xạ bumper vẫn bật.'),
        DeclareLaunchArgument(
            'software_render', default_value='false', choices=['true', 'false'],
            description='true = đặt LIBGL_ALWAYS_SOFTWARE=1 (render bằng CPU): chậm hơn nhưng '
                        'đúng. Dùng khi Gazebo crash/đen màn hình hoặc check_sensors.py báo '
                        'LiDAR/cliff sai (hay gặp trên WSL2 và máy driver đồ hoạ lỗi).'),
    ]


def render_env():
    """Phải đứng TRƯỚC mọi Node/ExecuteProcess để chúng nhận biến môi trường."""
    return [SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE', '1',
                                   condition=IfCondition(LaunchConfiguration('software_render')))]


def world_resource_env():
    """Cho Gazebo tìm thấy world của repo (worlds/gesture_arena.sdf).

    Launch của TurtleBot 4 GHI ĐÈ IGN_GAZEBO_RESOURCE_PATH (SetEnvironmentVariable), nên không thể thêm
    thư mục vào biến đó. Gazebo Fortress 6.16 (bản apt hiện tại của Humble) đọc thêm GZ_SIM_RESOURCE_PATH
    -> nối thư mục worlds/ của package vào biến này. Không ảnh hưởng các world có sẵn (maze, depot...)."""
    return [AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH',
                                      os.path.join(get_package_share_directory(PKG), 'worlds'))]


def rviz_node():
    return Node(
        package='rviz2', executable='rviz2', name='rviz2',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        parameters=[{'use_sim_time': True}], output='screen',
        condition=IfCondition(LaunchConfiguration('use_rviz')),
    )


def turtlebot4_sim(extra_param_actions=()):
    """Gazebo Fortress + TurtleBot 4 bằng launch chính thức (KHÔNG scope – xem lưu ý đầu file).

    extra_param_actions: thêm SetParametersFromFile... nạp SAU file mặc định (ghi đè được nó)."""
    pkg_lab = get_package_share_directory(PKG)
    pkg_tb4_ign = get_package_share_directory('turtlebot4_ignition_bringup')
    return GroupAction(scoped=False, actions=[
        *world_resource_env(),
        # Tham số motion_control của Create 3 phải đặt LÚC KHỞI ĐỘNG (đổi lúc chạy bị từ chối).
        # Khoá /**/motion_control trong file nên chỉ node đó dùng.
        SetParametersFromFile(
            os.path.join(pkg_lab, 'config', 'create3_no_cliff_reflex.yaml'),
            condition=UnlessCondition(LaunchConfiguration('cliff_reflex'))),
        *extra_param_actions,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_tb4_ign, 'launch', 'turtlebot4_ignition.launch.py')),
            launch_arguments={
                'world': LaunchConfiguration('world'),
                'model': LaunchConfiguration('model'),
                'x': LaunchConfiguration('x'),
                'y': LaunchConfiguration('y'),
                'z': LaunchConfiguration('z'),
                'yaw': LaunchConfiguration('yaw'),
                'namespace': '',
                'rviz': 'false',
                'slam': 'false',
                'localization': 'false',
                'nav2': 'false',
            }.items(),
        ),
    ])


def wait_for_sim_node():
    """Thoát khi /clock chạy và có /odom -> dùng làm mốc (OnProcessExit) để bật SLAM/AMCL/Nav2."""
    return Node(package=PKG, executable='wait_for_sim.py', name='tb4_lab04_wait_for_sim',
                output='screen')


def after_sim_ready(wait_node, actions):
    """Chạy `actions` (SLAM/AMCL/Nav2/auto_explore) khi wait_for_sim.py thoát.

    Bỏ qua nếu launch ĐANG TẮT (người dùng Ctrl+C lúc còn chờ Gazebo): nếu không, launch vẫn in
    "Khởi động SLAM Toolbox ..." và chờ hết các TimerAction (~20 s) rồi mới thoát."""
    actions = list(actions)

    def _on_exit(event, context):
        return [] if context.is_shutdown else actions

    return RegisterEventHandler(OnProcessExit(target_action=wait_node, on_exit=_on_exit))
