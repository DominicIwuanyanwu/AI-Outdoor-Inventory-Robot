import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    inventory_share = get_package_share_directory('inventory_sim')
    nav2_share = get_package_share_directory('nav2_bringup')

    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(inventory_share, 'launch', 'sim.launch.py')
        )
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_share, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': '/home/u790219/nav2_ws/src/inventory_sim/maps/my_map.yaml',
            'use_sim_time': 'true',
            'params_file': os.path.join(inventory_share, 'config', 'nav2_params.yaml'),
            'autostart': 'true'
        }.items()
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(inventory_share, 'rviz', 'nav2_view.rviz')],
        parameters=[{'use_sim_time': True}],  # ← FIX: RViz uses sim time
        output='screen'
    )

    return LaunchDescription([
        sim_launch,
        nav2_launch,
        rviz_node
    ])
