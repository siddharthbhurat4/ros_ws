from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mobile_robot_localization',  
            executable='controller_node', 
            name='rectangle_drive_controller',
            output='screen',
        #     parameters=[{'your_param_name': 'your_param_value'}],  # Replace with your actual parameters
        #     remappings=[('/old_topic', '/new_topic')],  # Replace with topic remappings if needed
        ),
        Node(
            package='mobile_robot_localization',
            executable='localization_node',  
            name='localization_gps_imu',
            output='screen',
        #     parameters=[{'your_param_name': 'your_param_value'}],  # Replace with your actual parameters
        #     remappings=[('/old_topic', '/new_topic')],  # Replace with topic remappings if needed
        )
    ])
