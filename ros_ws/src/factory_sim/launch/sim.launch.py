"""factory_sim の起動: gzserver (+gzclient) / robot_state_publisher / spawn。

引数:
  headless : true なら gzserver のみ（既定 false）
  world    : .world のパス（既定は install 先の worlds/factory_b1.world）
  cargo    : xacro 引数。true で積載箱を表示（既定 false）
  max_wheel_accel / max_wheel_torque / odom_source : diff_drive の暫定値（感度試験用）
  x, y     : ロボットの初期位置 [m]（B1 の store = 0, 0）
  clock_rate : /clock の配信周波数 [Hz]（gazebo_ros_init の publish_rate。既定 100 = 分解能 0.01 s。素の既定は 10）
  safety   : true なら安全層 safety_stop を起動（/cmd_vel_raw -> LiDAR で前方確認 -> /cmd_vel）。
             false なら起動しない（その場合は /cmd_vel に直接出す。waypoint_runner --direct）
"""
import os
import tempfile
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _spawn(context):
    x, y = LaunchConfiguration('x').perform(context), LaunchConfiguration('y').perform(context)
    return [Node(
        package='gazebo_ros', executable='spawn_entity.py', name='spawn_entity', output='screen',
        arguments=['-topic', 'robot_description', '-entity', 'factory_robot',
                   '-x', x, '-y', y, '-z', '0.02'],
        parameters=[{'use_sim_time': True}])]


def _resource_env():
    # 「Completed!」表示板の material/テクスチャ: share/factory_sim/media（gzserver/gzclient 両方に渡す。既存の値は残す）
    share = get_package_share_directory('factory_sim')
    return {'GAZEBO_RESOURCE_PATH': share + os.pathsep + os.environ.get('GAZEBO_RESOURCE_PATH', '/usr/share/gazebo-11')}


def _gzserver(context):
    # gzserver の `-p` は Gazebo 側の --play と解釈されるので、ROS パラメータはファイルで渡す
    # （gazebo_ros_init のノード名は gazebo、パラメータ publish_rate が /clock の周波数）
    rate = float(LaunchConfiguration('clock_rate').perform(context))
    fd, path = tempfile.mkstemp(prefix='gazebo_params_', suffix='.yaml')
    with os.fdopen(fd, 'w') as fh:
        fh.write('gazebo:\n  ros__parameters:\n    publish_rate: %s\n' % rate)
    plugins = ['-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so']
    # world の <plugin filename="libm5_visual_state.so"> は colcon が lib/factory_sim に置く（ランプ・積載箱・置き場の色）
    plugin_dir = os.path.join(get_package_prefix('factory_sim'), 'lib', 'factory_sim')
    env = {'GAZEBO_PLUGIN_PATH': plugin_dir + os.pathsep + os.environ.get('GAZEBO_PLUGIN_PATH', ''), **_resource_env()}
    return [ExecuteProcess(cmd=['gzserver', '--verbose', *plugins, LaunchConfiguration('world').perform(context),
                                '--ros-args', '--params-file', path], output='screen', additional_env=env)]


def generate_launch_description():
    share = get_package_share_directory('factory_sim')
    robot_description = ParameterValue(
        Command(['xacro ', PathJoinSubstitution([share, 'urdf', 'robot.urdf.xacro']),
                 ' cargo:=', LaunchConfiguration('cargo'),
                 ' max_wheel_accel:=', LaunchConfiguration('max_wheel_accel'),
                 ' max_wheel_torque:=', LaunchConfiguration('max_wheel_torque'),
                 ' odom_source:=', LaunchConfiguration('odom_source')]), value_type=str)
    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('world', default_value=os.path.join(share, 'worlds', 'factory_b1.world')),
        DeclareLaunchArgument('cargo', default_value='false'),
        DeclareLaunchArgument('max_wheel_accel', default_value='2.0'),
        DeclareLaunchArgument('max_wheel_torque', default_value='5.0'),
        DeclareLaunchArgument('odom_source', default_value='1'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('clock_rate', default_value='100.0'),
        DeclareLaunchArgument('safety', default_value='true'),
        OpaqueFunction(function=_gzserver),
        Node(package='factory_sim', executable='safety_stop.py', name='safety_stop', output='screen',
             parameters=[{'use_sim_time': True}], condition=IfCondition(LaunchConfiguration('safety'))),
        ExecuteProcess(cmd=['gzclient'], output='screen', additional_env=_resource_env(), condition=UnlessCondition(LaunchConfiguration('headless'))),
        Node(package='robot_state_publisher', executable='robot_state_publisher', output='screen',
             parameters=[{'use_sim_time': True, 'robot_description': robot_description}]),
        OpaqueFunction(function=_spawn),
    ])
