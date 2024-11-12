import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    urdf_file_name = 'urdf/mini_urdf.urdf'
    urdf = os.path.join(
        get_package_share_directory('mini_urdf'),
        urdf_file_name)
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_desc}],
            arguments=[urdf]),
        Node(
            package='urdf_lunabot',
            executable='state_publisher',
            name='state_publisher',
            output='screen'),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='body_axis_tf',
            output='screen',
            arguments=["0", "0", "0", "0", "0", "0", "/axis", "/base_link"]
        ),
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='robot_baselink_tf',
        #     output='screen',
        #     arguments=["0", "0", "0", "0", "0", "0", "/robot", "/base_link"]
        # ),
    ])