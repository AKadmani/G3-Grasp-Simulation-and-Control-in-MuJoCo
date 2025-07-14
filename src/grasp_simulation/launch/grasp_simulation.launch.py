from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('grasp_simulation')
    
    # Declare arguments
    grasp_type_arg = DeclareLaunchArgument(
        'grasp_type',
        default_value='power',
        description='Type of grasp: power, precision, or lateral'
    )
    
    control_mode_arg = DeclareLaunchArgument(
        'control_mode',
        default_value='impedance',
        description='Control mode: impedance, pid, or hybrid'
    )
    
    # Nodes
    grasp_controller_node = Node(
        package='grasp_simulation',
        executable='grasp_controller',
        name='grasp_controller',
        output='screen',
        parameters=[{
            'grasp_type': LaunchConfiguration('grasp_type'),
            'control_mode': LaunchConfiguration('control_mode'),
        }]
    )
    
    visualizer_node = Node(
        package='grasp_simulation',
        executable='mujoco_visualizer',
        name='mujoco_visualizer',
        output='screen'
    )
    
    return LaunchDescription([
        grasp_type_arg,
        control_mode_arg,
        grasp_controller_node,
        visualizer_node
    ])