from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
#from launch_ros.actions.node import SomeRemapRules
#from typing import Tuple
import os
#from launch_ros.remap_rule_type import SomeRemapRule


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
    #bob has been made instead
    #and no you don't wanna know
    #imuRemap = SomeRemapRule(Tuple(("/imu", "/imu0")))
    #cameraRemap = SomeRemapRule(["/color/image_raw", "/cam0/image_raw"])
    #nodeRemaps = SomeRemapRules([imuRemap, cameraRemap])

    """
    ov_msckf subs to 3 topics
    /cam0/image_raw [sensor_msgs/msg/Image]
    /cam1/image_raw [sensor_msgs/msg/Image]
    /imu0 [sensor_msgs/msg/Imu]
    """
    

    ld = LaunchDescription([
        Node( #realsense camera node
            package="realsense2_camera", #installed via realsense ros wrapper
            executable="realsense2_camera_node", #node found within the lib folder of the ros install
            name="camera"#, #name of node
        ),
        Node(
            package="openVinsTest",
            namespace="",
            executable="bob",
            name="bob_node"
        ),
        Node(
            package="rviz2",
            namespace="",
            executable="rviz2",
            name="rviz2"
        )
    ])
    ld.add_action(realsense_openvins)
    return ld
