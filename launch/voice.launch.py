# Bài 7 – chỉ 2 node giọng nói (dùng khi Gazebo + Nav2 ĐÃ chạy ở terminal khác: navigation.launch.py).
#
#   ros2 launch tb4_lab04 voice.launch.py                         # Vosk + micro + voice_navigator
#   ros2 launch tb4_lab04 voice.launch.py asr_engine:=whisper
#   ros2 launch tb4_lab04 voice.launch.py use_asr:=false          # chỉ voice_navigator (gõ lệnh bằng
#                                                                 #   voice_keyboard.py / ros2 topic pub)
#   ros2 launch tb4_lab04 voice.launch.py wav_files:=/tmp/a.wav,/tmp/b.wav   # nhận dạng file WAV thay micro
#
# Chưa có maps/zones.yaml -> launch tự chạy zone_segmentation.py trước (mất ~2 s).
# Tên tham số có tiền tố (asr_engine, mic_device ...) vì ở ROS 2 Humble các launch include nhau DÙNG CHUNG
# tham số – "device", "model" đã bị yolo.launch.py / launch TurtleBot 4 dùng.

import os
import sys

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PKG = 'tb4_lab04'


def _truthy(v):
    return v.strip().lower() in ('1', 'true', 'yes', 'on')


def ensure_zones(zones_file):
    """Đường dẫn zones.yaml (tạo bằng zone_segmentation.py nếu chưa có) hoặc (None, lỗi)."""
    scripts = os.path.join(get_package_prefix(PKG), 'lib', PKG)
    if scripts not in sys.path:
        sys.path.insert(0, scripts)
    import lab_common as lc
    if zones_file:
        path = os.path.abspath(os.path.expanduser(zones_file))
        return (path, None) if os.path.isfile(path) else (None, 'Không thấy file khu vực ' + path)
    path = lc.find_config('maps', 'zones.yaml')[0]
    if path:
        return path, None
    import zone_segmentation
    print('[tb4_lab04] Chưa có maps/zones.yaml -> chạy zone_segmentation.py ...', flush=True)
    code = zone_segmentation.main([])
    path = lc.find_config('maps', 'zones.yaml')[0]
    if path is None:
        return None, 'zone_segmentation.py thất bại (mã {}) – chạy tay để xem lỗi'.format(code)
    return path, None


def voice_actions(context):
    zones, err = ensure_zones(LaunchConfiguration('zones_file').perform(context).strip())
    if err:
        return [LogInfo(msg='\n[tb4_lab04] LỖI: ' + err + '\n'), Shutdown(reason=err)]
    params_file = LaunchConfiguration('voice_params_file').perform(context)
    nav_over = {'zones_file': zones, 'speak': _truthy(LaunchConfiguration('speak').perform(context))}
    asr_over = {'zones_file': zones}
    engine = LaunchConfiguration('asr_engine').perform(context).strip()
    if engine:
        asr_over['engine'] = engine
    mic = LaunchConfiguration('mic_device').perform(context).strip()
    if mic:
        asr_over['device'] = mic
    grammar = LaunchConfiguration('use_grammar').perform(context).strip()
    if grammar:
        asr_over['use_grammar'] = _truthy(grammar)
    wavs = [w.strip() for w in LaunchConfiguration('wav_files').perform(context).split(',') if w.strip()]
    if wavs:
        asr_over['wav_files'] = [os.path.abspath(os.path.expanduser(w)) for w in wavs]
    actions = [
        LogInfo(msg='[tb4_lab04] Khu vực: ' + zones),
        Node(package=PKG, executable='voice_navigator.py', name='tb4_voice_navigator', output='screen',
             emulate_tty=True, parameters=[params_file, nav_over]),
    ]
    if _truthy(LaunchConfiguration('use_asr').perform(context)):
        actions.append(Node(package=PKG, executable='speech_recognizer_node.py', name='tb4_speech_recognizer',
                            output='screen', emulate_tty=True, parameters=[params_file, asr_over]))
    return actions


def voice_arguments():
    pkg = get_package_share_directory(PKG)
    return [
        DeclareLaunchArgument('voice_params_file', default_value=os.path.join(pkg, 'config', 'voice.yaml'),
                              description='Tham số của speech_recognizer + voice_navigator'),
        DeclareLaunchArgument('zones_file', default_value='',
                              description='File khu vực (trống = maps/zones.yaml, tự tạo nếu chưa có)'),
        DeclareLaunchArgument('use_asr', default_value='true', choices=['true', 'false'],
                              description='Chạy node nhận dạng giọng nói (false = chỉ gõ lệnh)'),
        DeclareLaunchArgument('asr_engine', default_value='', description='vosk | whisper (trống = theo file)'),
        DeclareLaunchArgument('mic_device', default_value='', description='Tên/số micro (trống = mặc định)'),
        DeclareLaunchArgument('use_grammar', default_value='', description='true/false – Vosk grammar'),
        DeclareLaunchArgument('wav_files', default_value='',
                              description='Danh sách file WAV (phẩy ngăn cách) thay cho micro'),
        DeclareLaunchArgument('speak', default_value='false', choices=['true', 'false'],
                              description='Đọc phản hồi bằng espeak-ng'),
    ]


def generate_launch_description():
    return LaunchDescription(voice_arguments() + [OpaqueFunction(function=voice_actions)])
