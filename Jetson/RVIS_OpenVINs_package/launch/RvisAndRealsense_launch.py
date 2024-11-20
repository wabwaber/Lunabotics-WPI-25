from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
import os

openvins_msckf_launch_path = "/home/jetson/catkin_ws/src/open_vins/ov_msckf/launch"

def generate_launch_description():
    realsense_openvins = GroupAction(
        [
            IncludeLaunchDescription( #openvins
                PythonLaunchDescriptionSource(
                    os.path.join(openvins_msckf_launch_path, "subscribe.launch.py")
                )
            )
        ]
    )

    """
    ov_msckf subs to 3 topics
    /cam0/image_raw [sensor_msgs/msg/Image]
    /cam1/image_raw [sensor_msgs/msg/Image]
    /imu0 [sensor_msgs/msg/Imu]
    """

    ld = LaunchDescription([
        Node( #realsense camera node
            package='realsense2_camera', #installed via realsense ros wrapper
            executable='realsense2_camera_node', #node found within the lib folder of the ros install
            name='camera' #name of node
        ),
        Node(
            package='turtlesim',
            executable='mimic',
            name='mimic',
            remappings=[
                ('', '')
                ('/imu', '/imu0')
            ]
        ),
        Node(
            package='rviz2',
            namespace='',
            executable='rviz2',
            name='rviz2'
        )
    ])
    ld.add_action(realsense_openvins)
    return ld