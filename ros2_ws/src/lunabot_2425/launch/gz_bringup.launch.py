import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

def generate_launch_description():

    package_name = "lunabot_2425"

    ### DECLARE LAUNCH ARGUMENTS

    # ...

    ### INCLUDE LAUNCH FILES

    # Create the robot state publisher
    rsp_source = PythonLaunchDescriptionSource(os.path.join(
        get_package_share_directory("lunabot_2425"),
        "launch",
        "rsp.launch.py"
    ))
    
    rsp = IncludeLaunchDescription(
        rsp_source,
        launch_arguments={
            "use_sim_time": "true"
        }.items()
    )

    gz_sim_source = PythonLaunchDescriptionSource(os.path.join(
        get_package_share_directory("ros_gz_sim"), 
        "launch", 
        "gz_sim.launch.py"
    ))

    gz_sim = IncludeLaunchDescription(
        gz_sim_source,
        launch_arguments={
            'gz_args': "-r empty.sdf",
            'on_exit_shutdown': 'True'
        }.items(),
    )

    # DEFINE NODES

    gz_create_robot = Node(
        package = "ros_gz_sim",
        executable="create",
        arguments=["-topic", "robot_description",
                   "-name", "mooncake"],
        output="screen",
    )

    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[
            {"config_file": os.path.join(
                get_package_share_directory("lunabot_2425"),
                "config",
                "gz_bridge.config.yaml",
            )}
        ],
        # remappings=[],
        output='screen'
    )

    return LaunchDescription([
        rsp,
        gz_sim,
        gz_create_robot,
        gz_bridge
    ])