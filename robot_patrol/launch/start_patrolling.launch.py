import os                                                        
from ament_index_python.packages import get_package_share_directory
import launch
import launch_ros.actions

def generate_launch_description():

    rviz_config = os.path.join(
        get_package_share_directory('robot_patrol'), 'rviz', 'patrol.rviz')

    # Nodes
    patrol_node = launch_ros.actions.Node(
        package='robot_patrol',
        executable='patrol_executable',
        output='screen',
    )
    
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
    )

    return launch.LaunchDescription([
        patrol_node,
        rviz_node,
    ])