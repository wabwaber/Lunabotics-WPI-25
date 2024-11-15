import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    # gz_launch_path = os.join(
        # pkg_ros_gz_sim, 'launch/gz_sim.launch.py')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    urdf_file_name = 'urdf/mini_urdf.urdf'
    rviz_file_name = 'demo.rviz'
    urdf = os.path.join(
        get_package_share_directory('mini_urdf'),
        urdf_file_name)
    rviz = os.path.join(
        get_package_share_directory('lunabot_demo'),
        rviz_file_name)
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
            package='lunabot_demo',
            executable='state_publisher',
            name='state_publisher',
            output='screen'),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='robot_odom_tf',
            output='screen',
            arguments=["0", "0", "0", "0", "0", "0", "/axis", "/robot"]
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='robot_baselink_tf',
            output='screen',
            arguments=["0", "0", "0", "0", "0", "0", "/robot", "/base_link"]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=["-d", [rviz]]
        ),
        # IncludeLaunchDescription(
        #     LaunchDescriptionSource(gz_launch_path),
        #     launch_arguments={
        #         'gz_args': [PathJoinSubstitution([pkg_spaceros_gz_sim, 'worlds',
        #                                           LaunchConfiguration('world_file')])],
        #         'on_exit_shutdown': 'True'
        #     }.items(),
        # ),
        # Node(
        #     package='gazebo_ros', 
        #     executable='spawn_entity.py',
        #     arguments=['-entity', "mini_urdf", 
        #         '-topic', 'robot_description',
        #             '-x', 0,
        #             '-y', 0,
        #             '-z', 0,
        #             '-Y', 0],
        #             output='screen')
    ])