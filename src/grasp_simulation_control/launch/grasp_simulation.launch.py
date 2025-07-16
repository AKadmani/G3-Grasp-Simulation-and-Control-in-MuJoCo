from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare launch arguments
    controller_type_arg = DeclareLaunchArgument(
        'controller_type',
        default_value='impedance',
        description='Controller type: pid, impedance, hybrid, or adaptive'
    )
    
    object_type_arg = DeclareLaunchArgument(
        'object_type',
        default_value='cylinder',
        description='Object type: cylinder, box, or sphere'
    )
    
    grasp_type_arg = DeclareLaunchArgument(
        'grasp_type',
        default_value='cylindrical',
        description='Grasp type: power, precision, lateral, spherical, or cylindrical'
    )
    
    contact_model_arg = DeclareLaunchArgument(
        'contact_model',
        default_value='SF',
        description='Contact model: SF (Soft Finger), HF (Hard Finger), or FF (Frictionless)'
    )
    
    scene_file_arg = DeclareLaunchArgument(
        'scene_file',
        default_value='scene_grasp.xml',
        description='MuJoCo scene XML file'
    )
    
    # Create node
    grasp_simulation_node = Node(
        package='grasp_simulation_control',
        executable='grasp_simulation_node.py',
        name='grasp_simulation',
        parameters=[{
            'controller_type': LaunchConfiguration('controller_type'),
            'object_type': LaunchConfiguration('object_type'),
            'grasp_type': LaunchConfiguration('grasp_type'),
            'contact_model': LaunchConfiguration('contact_model'),
            'scene_file': LaunchConfiguration('scene_file')
        }],
        output='screen'
    )
    
    # Optional: RViz for visualization
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', 'grasp_visualization.rviz'],
        condition=None  # Can add condition to make optional
    )
    
    return LaunchDescription([
        controller_type_arg,
        object_type_arg,
        grasp_type_arg,
        contact_model_arg,
        scene_file_arg,
        grasp_simulation_node,
        # rviz_node  # Uncomment if you want RViz
    ])