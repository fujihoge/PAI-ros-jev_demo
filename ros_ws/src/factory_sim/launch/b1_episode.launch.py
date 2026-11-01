"""Task B1 replay episode: sim.launch.py (worlds/factory_b1.world, robot, safety layer) + b1_node.

  ros2 launch factory_sim b1_episode.launch.py headless:=false out:=out/b1_replay
Arguments: decider (node | inprocess), policy (replay | jev_b1x), decision_timeout_s, trace (recorded decisions, default recorded/b1x_seed5_empty.json), duration [s], pace (true/false), out, headless, confirm_s, nav_wmax, nav_vmax, nav_drive_dec, nav_turn_dec, max_wheel_accel, max_wheel_torque.
The node ends the launch when the episode is saved.
"""
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription, OpaqueFunction,
                            RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _node(context):
    g = lambda k: LaunchConfiguration(k).perform(context)  # noqa: E731
    params = {'use_sim_time': True, 'duration': float(g('duration')), 'out': g('out'), 'pace': g('pace').lower() == 'true',
              'seed': int(g('seed')), 'nav_confirm_s': float(g('confirm_s')), 'nav_wmax': float(g('nav_wmax')),
              'nav_turn_dec': float(g('nav_turn_dec')),
              'nav_vmax': float(g('nav_vmax')), 'nav_drive_dec': float(g('nav_drive_dec'))}
    if g('trace'):
        params['trace'] = g('trace')
    params['decider'] = g('decider')
    params['decision_timeout_s'] = float(g('decision_timeout_s'))
    params['debug_decisions'] = g('debug_decisions').lower() == 'true'
    node = Node(package='factory_sim', executable='b1_node.py', name='b1_node', output='screen', parameters=[params])
    actions = [node, RegisterEventHandler(OnProcessExit(target_action=node, on_exit=[EmitEvent(event=Shutdown())]))]
    if g('decider') == 'node':
        dparams = {'use_sim_time': True, 'policy': g('policy'), 'jev_transport': g('jev_transport'),
                   'jev_timeout_s': float(g('jev_timeout_s'))}
        if g('trace'):
            dparams['trace'] = g('trace')
        if g('jev_sdk_path'):
            dparams['jev_sdk_path'] = g('jev_sdk_path')
        actions.insert(0, Node(package='factory_sim', executable='decision_node.py', name='decision_node', output='screen',
                               parameters=[dparams]))
    return actions


def generate_launch_description():
    share = FindPackageShare('factory_sim')
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([share, 'launch', 'sim.launch.py'])),
        launch_arguments={'headless': LaunchConfiguration('headless'), 'cargo': 'true',
                          'clock_rate': LaunchConfiguration('clock_rate'),
                          'safety': LaunchConfiguration('safety'), 'max_wheel_accel': LaunchConfiguration('max_wheel_accel'),
                          'max_wheel_torque': LaunchConfiguration('max_wheel_torque'), 'world': LaunchConfiguration('world')}.items())
    return LaunchDescription([
        DeclareLaunchArgument('trace', default_value=''),
        DeclareLaunchArgument('duration', default_value='1800'),
        DeclareLaunchArgument('seed', default_value='5'),
        DeclareLaunchArgument('pace', default_value='true'),
        DeclareLaunchArgument('headless', default_value='true'),
        DeclareLaunchArgument('out', default_value='out/b1_replay'),
        DeclareLaunchArgument('confirm_s', default_value='0.2'),
        DeclareLaunchArgument('nav_wmax', default_value='1.0'),
        DeclareLaunchArgument('nav_turn_dec', default_value='1.5'),
        DeclareLaunchArgument('nav_vmax', default_value='0.6'),
        DeclareLaunchArgument('nav_drive_dec', default_value='0.8'),
        DeclareLaunchArgument('max_wheel_accel', default_value='2.0'),
        DeclareLaunchArgument('max_wheel_torque', default_value='5.0'),
        DeclareLaunchArgument('decider', default_value='node'),               # node: decision_node decides | inprocess: inside b1_node
        DeclareLaunchArgument('debug_decisions', default_value='false'),
        DeclareLaunchArgument('policy', default_value='replay'),              # decision_node: replay | jev_b1x
        DeclareLaunchArgument('decision_timeout_s', default_value='8.0'),     # b1_node: wall seconds to wait for an answer, then timeout fallback (smallest slack)
        DeclareLaunchArgument('jev_transport', default_value='mock'),         # decision_node, policy:=jev_b1x: mock (no network) | sdk (real API)
        DeclareLaunchArgument('jev_timeout_s', default_value='10.0'),
        DeclareLaunchArgument('jev_sdk_path', default_value=''),
        DeclareLaunchArgument('clock_rate', default_value='100.0'),
        DeclareLaunchArgument('safety', default_value='true'),
        DeclareLaunchArgument('world', default_value=PathJoinSubstitution([share, 'worlds', 'factory_b1.world'])),
        sim,
        OpaqueFunction(function=_node),
    ])
