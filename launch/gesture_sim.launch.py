# Bài 6 – Phần 1: TurtleBot 4 trong Gazebo (Ignition Fortress), world "gesture_arena" của repo:
# sân phẳng 9 m x 9 m, không vật cản bên trong -> đủ rộng cho mọi hành vi cử chỉ, không va chạm.
#
#   ros2 launch tb4_lab04 gesture_sim.launch.py
#   ros2 launch tb4_lab04 gesture_sim.launch.py world:=maze          # world của bài trước
#   ros2 launch tb4_lab04 gesture_sim.launch.py use_rviz:=true software_render:=true
#
# Kiểm tra robot nhận /cmd_vel (sau khi Gazebo lên, robot đang ở dock):
#   ros2 action send_goal /undock irobot_create_msgs/action/Undock "{}"
#   ros2 topic pub -r 10 -t 30 /cmd_vel geometry_msgs/msg/Twist "{angular: {z: 0.5}}"
# hoặc: ros2 run tb4_lab04 check_cmd_vel.py

import os
import sys

from launch import LaunchDescription

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import tb4_lab04_common as common  # noqa: E402


def generate_launch_description():
    abort = common.abort_if_missing(common.PKGS_SIM)
    if abort is not None:
        return abort
    return LaunchDescription(
        common.sim_arguments('sim.rviz', world_note='World Gazebo (gesture_arena = sân trống của Bài 6)',
                             default_world='gesture_arena', default_rviz='false')
        + common.render_env()
        + [common.rviz_node(), common.turtlebot4_sim()]
    )
