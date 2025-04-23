from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
#from launch_ros.actions.node import SomeRemapRules
#from typing import Tuple
import os
#from launch_ros.remap_rule_type import SomeRemapRule

openvins_msckf_launch_path = "/catkin_ws/open_vins/ov_msckf/launch"
realsense_launch_path="/opt/ros/humble/share/realsense2_camera/launch"
openvins_config_path = "/catkin_ws/Lunabotics-WPI-25/Jetson/src/openVinsTest/estimator_config.yaml"

def generate_launch_description():
    realsense_openvins = GroupAction(
        [
            IncludeLaunchDescription( #openvins
                PythonLaunchDescriptionSource(
                    os.path.join(openvins_msckf_launch_path, "subscribe.launch.py")
                ),
                launch_arguments={
                    #'verbosity': 'ALL',
                    'config': 'rs_d455',
                    #'config_path': '/catkin_ws/Lunabotics-WPI-25/Jetson/src/openVinsTest/estimator_config.yaml',
                    'rviz_enable': 'true',
                    #'max_cameras': '1',
                    #'use_stereo': 'false',
                    #'save_total_state': 'true',
                }.items(),
            )
        ]
    )
    realsense_launch= GroupAction([
            IncludeLaunchDescription( #realsense
                PythonLaunchDescriptionSource(
                    os.path.join(realsense_launch_path, "rs_launch.py")
                ),
                launch_arguments={ #NOTE you can either have the IMU or the Cameras enabled at a time if running on a USB 2.1 port however the cable that is currently being used for the realsense is a 3.2 cable so things should be all good
                    'enable_accel': 'true',
                    'enable_gyro': 'true',
                    'accel_fps': '200', 
                    'unite_imu_method': '2',
                    'enable_color': 'true',
                    'enable_infra': 'true',
                    'colorizer.enable': 'true',
                    'hole_filling_filter.enable': 'false',
                    'enable_sync': 'true', #syncs the images publish times (TESTING)
                    'align_depth': 'true',
                    'align_depth.enable': 'true',
                    'enable_rgbd': 'true',
                    #'pointcloud.enable': 'true',
                    'device_type': 'd455',
                    #'initial_reset': 'true',
                    #'serial_no': 213622253386, #commented out because of a contradictory error, go ahead and uncomment it if you want to see why
                }.items(),
            )
        ])

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

    #Rvis is launched by OpenVINS see launch arguments above on or near line 24
    ld = LaunchDescription([
        Node(
            package="openVinsTest",
            namespace="",
            executable="bob",
            name="bob_node"
        )
        # Node(
        #     package="rviz2",
        #     namespace="",
        #     executable="rviz2",
        #     name="rviz2"
        # )
    ])
    ld.add_action(realsense_launch)
    ld.add_action(realsense_openvins)
    return ld

"""
Below is a full list of realsense_camera parameters (taken from ros2 param list with the node running):
camera/camera:
  accel_fps
  accel_info_qos
  accel_qos
  align_depth.enable
  align_depth.frames_queue_size
  angular_velocity_cov
  base_frame_id
  camera.color.image_raw.enable_pub_plugins
  camera.depth.image_rect_raw.enable_pub_plugins
  camera.infra1.image_rect_raw.enable_pub_plugins
  camera_name
  clip_distance
  color_info_qos
  color_qos
  colorizer.color_scheme
  colorizer.enable
  colorizer.frames_queue_size
  colorizer.histogram_equalization_enabled
  colorizer.max_distance
  colorizer.min_distance
  colorizer.stream_filter
  colorizer.stream_format_filter
  colorizer.stream_index_filter
  colorizer.visual_preset
  decimation_filter.enable
  decimation_filter.filter_magnitude
  decimation_filter.frames_queue_size
  decimation_filter.stream_filter
  decimation_filter.stream_format_filter
  decimation_filter.stream_index_filter
  depth_info_qos
  depth_module.auto_exposure_limit
  depth_module.auto_exposure_limit_toggle
  depth_module.auto_exposure_mode
  depth_module.auto_exposure_roi.bottom
  depth_module.auto_exposure_roi.left
  depth_module.auto_exposure_roi.right
  depth_module.auto_exposure_roi.top
  depth_module.auto_gain_limit
  depth_module.auto_gain_limit_toggle
  depth_module.depth_format
  depth_module.depth_profile
  depth_module.emitter_always_on
  depth_module.emitter_enabled
  depth_module.emitter_frequency
  depth_module.emitter_on_off
  depth_module.enable_auto_exposure
  depth_module.error_polling_enabled
  depth_module.exposure
  depth_module.frames_queue_size
  depth_module.gain
  depth_module.global_time_enabled
  depth_module.hdr_enabled
  depth_module.infra1_format
  depth_module.infra_format
  depth_module.infra_profile
  depth_module.inter_cam_sync_mode
  depth_module.laser_power
  depth_module.output_trigger_enabled
  depth_module.sequence_id
  depth_module.sequence_name
  depth_module.sequence_size
  depth_module.thermal_compensation
  depth_module.visual_preset
  depth_qos
  device_type
  diagnostics_period
  disparity_filter.enable
  disparity_to_depth.enable
  enable_accel
  enable_color
  enable_depth
  enable_gyro
  enable_infra
  enable_infra1
  enable_rgbd
  enable_sync
  filter_by_sequence_id.enable
  filter_by_sequence_id.frames_queue_size
  filter_by_sequence_id.sequence_id
  gyro_fps
  gyro_info_qos
  gyro_qos
  hdr_merge.enable
  hdr_merge.frames_queue_size
  hold_back_imu_for_frames
  hole_filling_filter.enable
  hole_filling_filter.frames_queue_size
  hole_filling_filter.holes_fill
  hole_filling_filter.stream_filter
  hole_filling_filter.stream_format_filter
  hole_filling_filter.stream_index_filter
  infra1_info_qos
  infra1_qos
  infra_info_qos
  infra_qos
  initial_reset
  json_file_path
  linear_accel_cov
  motion_module.enable_motion_correction
  motion_module.frames_queue_size
  motion_module.global_time_enabled
  motion_module.gyro_sensitivity
  pointcloud.allow_no_texture_points
  pointcloud.enable
  pointcloud.filter_magnitude
  pointcloud.frames_queue_size
  pointcloud.ordered_pc
  pointcloud.pointcloud_qos
  pointcloud.stream_filter
  pointcloud.stream_format_filter
  pointcloud.stream_index_filter
  publish_tf
  qos_overrides./parameter_events.publisher.depth
  qos_overrides./parameter_events.publisher.durability
  qos_overrides./parameter_events.publisher.history
  qos_overrides./parameter_events.publisher.reliability
  reconnect_timeout
  rgb_camera.auto_exposure_priority
  rgb_camera.auto_exposure_roi.bottom
  rgb_camera.auto_exposure_roi.left
  rgb_camera.auto_exposure_roi.right
  rgb_camera.auto_exposure_roi.top
  rgb_camera.backlight_compensation
  rgb_camera.brightness
  rgb_camera.color_format
  rgb_camera.color_profile
  rgb_camera.contrast
  rgb_camera.enable_auto_exposure
  rgb_camera.enable_auto_white_balance
  rgb_camera.exposure
  rgb_camera.frames_queue_size
  rgb_camera.gain
  rgb_camera.gamma
  rgb_camera.global_time_enabled
  rgb_camera.hue
  rgb_camera.power_line_frequency
  rgb_camera.saturation
  rgb_camera.sharpness
  rgb_camera.white_balance
  rosbag_filename
  serial_no
  spatial_filter.enable
  spatial_filter.filter_magnitude
  spatial_filter.filter_smooth_alpha
  spatial_filter.filter_smooth_delta
  spatial_filter.frames_queue_size
  spatial_filter.holes_fill
  spatial_filter.stream_filter
  spatial_filter.stream_format_filter
  spatial_filter.stream_index_filter
  start_type_description_service
  temporal_filter.enable
  temporal_filter.filter_smooth_alpha
  temporal_filter.filter_smooth_delta
  temporal_filter.frames_queue_size
  temporal_filter.holes_fill
  temporal_filter.stream_filter
  temporal_filter.stream_format_filter
  temporal_filter.stream_index_filter
  tf_publish_rate
  unite_imu_method
  usb_port_id
  use_sim_time
  wait_for_device_timeout

"""
