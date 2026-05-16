import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    inventory_share = get_package_share_directory('inventory_sim')
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(inventory_share, 'launch', 'sim.launch.py')
        )
    )
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(inventory_share, 'launch', 'slam.launch.py')
        ),
        launch_arguments={'use_sim_time': 'true'}.items()
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(inventory_share, 'rviz', 'nav2_view.rviz')],
        output='screen'
    )
    return LaunchDescription([sim_launch, slam_launch, rviz_node])
