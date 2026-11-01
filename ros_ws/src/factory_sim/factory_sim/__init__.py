"""factory_sim: ROS 2 / Gazebo integration of task B1.

Modules without rclpy (unit-testable): paths, colors, messages, nav_driver, nav_types, call_limit, b1_runner, b1_flow,
b1_visuals, decision_core. ROS nodes: b1_node, decision_node. `factory_b`, `policies` and `adapters` are imported
from the repository root (never copied).
"""
from . import paths as _paths

_paths.ensure_importable()
