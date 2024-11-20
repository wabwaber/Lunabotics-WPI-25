
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Start RViz2 automatically with this launch file.",
        )
    )

    # Initialize Arguments
    gui = LaunchConfiguration("gui")

    # get URDF file name
    robot_description_filename = PathJoinSubstitution([FindPackageShare('luna_urdf'), 'luna_urdf', 'robot.urdf'])
    
    # get content of URDF
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="cat")]),
            " ",
            robot_description_filename,
        ]
    )
    
    robot_controllers = PathJoinSubstitution([FindPackageShare("luna_urdf"), "luna_control", "controllers.yaml",])
    rviz_config_file = PathJoinSubstitution([FindPackageShare("lunabotdemo"), "demo.rviz"])

    gazebo_launch = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([PathJoinSubstitution(
                    [FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])]),
                launch_arguments={
                    'gz_args': [PathJoinSubstitution([FindPackageShare("luna_gz"), 'worlds',
                                                    "boxes.sdf"])],
                    'on_exit_shutdown': 'True'
                }.items(),
             )

    gazebo_spawn_lunabot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=["-file", robot_description_filename],
        output="screen",
    )
    
    # gazebo_set_model_path = SetEnvironmentVariable(name='GZ_SIM_RESOURCE_PATH', value=PathJoinSubstitution([FindPackageShare('luna_urdf'), 'luna_sdf']))

    gazebo_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[
            {"config_file": PathJoinSubstitution([FindPackageShare("lunabotdemo"), "gz_bridge.config.yaml"])},
        ],
        # remappings=[],
        output='screen'
    )

    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[
            {'use_sim_time': True},
            {'robot_description': robot_description_content},
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(gui),
    )

    joint_broadcast_spawner_node = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broadcast"],
    )

    drive_ctrl_spawner_node = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["drive_ctrl"],
    )

    nodes = [
        rsp_node,
        gazebo_launch,
        gazebo_spawn_lunabot,
        gazebo_bridge,
        rviz_node,
        joint_broadcast_spawner_node,
        drive_ctrl_spawner_node,
    ]

    return LaunchDescription(declared_arguments + nodes)