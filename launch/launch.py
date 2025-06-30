from launch.launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Launch left arm
    left_arm = IncludeLaunchDescription(
        PythonLaunchDescriptionSource('path/to/arm.launch.py'),
        launch_arguments={'arm_name': 'left_arm'}.items()
    )
    
    # Launch right arm  
    right_arm = IncludeLaunchDescription(
        PythonLaunchDescriptionSource('path/to/arm.launch.py'),
        launch_arguments={'arm_name': 'right_arm'}.items()
    )
    
    # Could add other nodes like rviz, static transforms, etc.
    
    return LaunchDescription([left_arm, right_arm])