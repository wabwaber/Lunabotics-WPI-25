import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.actions import SetLaunchConfiguration, Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    # pkg_luna_gz = get_package_share_directory('luna_gz')
    pkg_luna_urdf = get_package_share_directory('luna_urdf')
    gz_launch_path = PathJoinSubstitution([pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'])
    gz_model_path = PathJoinSubstitution([pkg_luna_urdf, 'luna_urdf'])

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    urdf_file_name = 'robot.urdf'
    urdf = os.path.join(
        get_package_share_directory('luna_urdf'),
        'luna_urdf',
        urdf_file_name)
    # sdf_file_name = 'robot.sdf'
    # sdf = os.path.join(
    #     get_package_share_directory('luna_urdf'),
    #     'luna_sdf',
    #     sdf_file_name)
    rviz_file_name = 'demo.rviz'
    rviz = os.path.join(
        get_package_share_directory('lunabot_demo'),
        rviz_file_name)
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),
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
            arguments=['0', '0', '0', '0', '0', '0', '/axis', '/robot']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='robot_baselink_tf',
            output='screen',
            arguments=['0', '0', '0', '0', '0', '0', '/robot', '/base_link']
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', [rviz]],
            on_exit=Shutdown()
        ),
        DeclareLaunchArgument(
            'world',
            default_value='empty',
            choices=['empty', 'moon', 'mars', 'enceladus'],
            description='World to load into Gazebo'
        ),
        SetLaunchConfiguration(name='world_file',
                               value=[LaunchConfiguration('world'),
                                      TextSubstitution(text='.sdf')]),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', gz_model_path),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gz_launch_path),
            launch_arguments={
                # 'gz_args': [PathJoinSubstitution([pkg_luna_gz, 'worlds',
                #                                   LaunchConfiguration('world_file')])],
                'gz_args': ['empty.sdf'],
                'on_exit_shutdown': 'True'
            }.items(),
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[],
            remappings=[],
            output='screen'
        ),
    ])
