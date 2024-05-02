from launch import LaunchDescription
from launch_ros.actions import Node
 
def generate_launch_description():
    return LaunchDescription([
        Node(
            package='soro_control',
            executable='trajectory_node.py',
            name='trajectory_controller',
            output="screen",
        ),
        Node(
            package='soro_control',
            executable='high_level_control_node.py',
            name='hl_controller',
            output="screen",
        ),
        Node(
            package='soro_control',
            executable='low_level_control_node.py',
            name='ll_controller',
            output="screen",
        ),
    ])
