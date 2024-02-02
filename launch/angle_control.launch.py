from launch import LaunchDescription
from launch_ros.actions import Node
 
def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joy',
            executable='joy_node',
            name='ps4_joy',
            output="screen",
        ),
        Node(
            package='soro_control',
            executable='ik_control_node.py',
            name='ik_controller',
            output="screen",
        ),
    ])
