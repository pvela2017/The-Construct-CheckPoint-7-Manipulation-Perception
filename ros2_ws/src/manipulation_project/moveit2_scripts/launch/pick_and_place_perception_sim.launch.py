import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("name", package_name="my_moveit_config").to_moveit_configs()
    package_name = "moveit2_scripts"

    perc = Node(
        package='simple_grasping',
        executable='basic_grasping_perception_node',
        name='basic_grasping_perception_node',
        output='screen',
        parameters=[{'debug_topics': True}, {'use_sim_time': True},],
        remappings=[('/depth/points' , '/wrist_rgbd_depth_sensor/points')],  # Add remappings if necessary
    )

    ppp_node = Node(
        name="pick_and_place_perception_sim",
        package="moveit2_scripts",
        executable="pick_and_place_perception_sim",
        output="screen",
        parameters=[moveit_config.robot_description,
                    moveit_config.robot_description_semantic,
                    moveit_config.robot_description_kinematics,
                    {'use_sim_time': True},],
    )

    # RVIZ config
    rviz_config_dir = os.path.join(get_package_share_directory(package_name), 'rviz', 'rviz2.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        name='rviz_node',
        parameters=[{'use_sim_time': True}],
        arguments=['-d', rviz_config_dir]
        )
    
    return LaunchDescription([
        perc,
        ppp_node,
        rviz_node
    ])
