# Bài thực hành 04 – Phần 1: TurtleBot 4 trong Ignition Gazebo (Fortress) + RViz2.
#
# Dùng nguyên launch chính thức turtlebot4_ignition_bringup/turtlebot4_ignition.launch.py
# (không tự viết lại simulator/controller), chỉ thay RViz bằng cấu hình fixed frame = odom
# vì ở Phần 1 chưa có frame "map".
#
#   ros2 launch tb4_lab04 sim.launch.py                         # world mặc định: maze
#   ros2 launch tb4_lab04 sim.launch.py world:=depot            # maze | depot | warehouse
#   ros2 launch tb4_lab04 sim.launch.py software_render:=true   # WSL2 / driver đồ hoạ lỗi
#
# Phần dùng chung (tham số, include TurtleBot 4, lưu ý kỹ thuật) nằm trong tb4_lab04_common.py.

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
        common.sim_arguments('sim.rviz')
        + common.render_env()
        # RViz đặt TRƯỚC include để condition của nó được đánh giá trước khi launch của
        # TurtleBot 4 ghi thêm biến vào context.
        + [common.rviz_node(), common.turtlebot4_sim()]
    )
