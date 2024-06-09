import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import get_package_prefix
from launch_ros.actions import Node


def generate_launch_description():
    package_name = "get_cube_pose"
    #cartographer_config_dir = os.path.join(get_package_share_directory('cartographer_slam'), 'config')
    #configuration_basename = 'cartographer.lua'

    perc = Node(
        package='simple_grasping',
        executable='basic_grasping_perception_node',
        name='basic_grasping_perception_node',
        output='screen',
        parameters=[{'debug_topics': True}],
        remappings=[('/depth/points' , '/camera/depth/color/points')]
    )

    get_pose = Node(
        package='get_cube_pose',
        executable='get_pose_client_node',
        name='get_pose_client_node',
        output='screen',
    )


    # RVIZ config
    rviz_config_dir = os.path.join(get_package_share_directory(package_name), 'rviz', 'rviz2.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        name='rviz_node',
        parameters=[{'use_sim_time': False}],
        arguments=['-d', rviz_config_dir]
        )
    
    return LaunchDescription([
        perc,
        get_pose,
        rviz_node
    ])