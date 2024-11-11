import rospy
import tf2_ros
import tf.transformations, tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, TransformStamped
from std_msgs.msg import Header
from std_msgs.msg import Float32
import math

# Random Ian Variables
l_magic = 0.127741
theta_magic = -1.87367


def update_localizer_angle_cb(localizer_angle_msg):
    global localizer_angle
    localizer_angle = localizer_angle_msg.data

def multiply_transforms(transform1, transform2):
    # Extract translation vectors
    translation1 = [transform1.transform.translation.x,
                    transform1.transform.translation.y,
                    transform1.transform.translation.z]
    translation2 = [transform2.transform.translation.x,
                    transform2.transform.translation.y,
                    transform2.transform.translation.z]

    # Extract rotation quaternions
    rotation1 = [transform1.transform.rotation.x,
                 transform1.transform.rotation.y,
                 transform1.transform.rotation.z,
                 transform1.transform.rotation.w]
    rotation2 = [transform2.transform.rotation.x,
                 transform2.transform.rotation.y,
                 transform2.transform.rotation.z,
                 transform2.transform.rotation.w]

    # Perform quaternion multiplication for rotation
    rotation_result = tf.transformations.quaternion_multiply(rotation1, rotation2)

    # Perform vector addition for translation
    translation_result = [translation1[i] + translation2[i] for i in range(3)]

    # Create a new TransformStamped message for the result
    result_transform = TransformStamped()
    result_transform.header.stamp = rospy.Time.now()
    result_transform.header.frame_id = transform1.header.frame_id
    result_transform.child_frame_id = transform2.child_frame_id
    result_transform.transform.translation.x = translation_result[0]
    result_transform.transform.translation.y = translation_result[1]
    result_transform.transform.translation.z = translation_result[2]
    result_transform.transform.rotation.x = rotation_result[0]
    result_transform.transform.rotation.y = rotation_result[1]
    result_transform.transform.rotation.z = rotation_result[2]
    result_transform.transform.rotation.w = rotation_result[3]

    return result_transform

if __name__ == '__main__':
    rospy.init_node('tf_solver')
    rospy.Subscriber('/jetson/localizer_angle', Float32, update_localizer_angle_cb)
    broadcaster = tf2_ros.StaticTransformBroadcaster()

    localizer_angle = 0.0

    # Define last_transform to be none
    last_transform = None

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    # Define tf between (0, 0, 0) of World Frame and Aruco Marker
    world_2_aruco = TransformStamped()
    world_2_aruco.header.frame_id = "world"
    world_2_aruco.child_frame_id = "aruco"
    world_2_aruco.header.stamp = rospy.Time.now()
    world_2_aruco.transform.translation.x = 0.0
    world_2_aruco.transform.translation.y = 0.63
    world_2_aruco.transform.translation.z = 0.0
    world_2_aruco.transform.rotation.x = 0.0
    world_2_aruco.transform.rotation.y = 0.0
    world_2_aruco.transform.rotation.z = 0.0
    world_2_aruco.transform.rotation.w = 1.0

    # Define tf between webcam and robot
    webcam_2_robot = TransformStamped()
    webcam_2_robot.header.frame_id = "webcam"
    webcam_2_robot.child_frame_id = "robot_unfused"
    webcam_2_robot.header.stamp = rospy.Time.now()
    webcam_2_robot.transform.translation.x = -0.0381
    webcam_2_robot.transform.translation.y = -0.12192
    webcam_2_robot.transform.translation.z = 0.0
    webcam_2_robot.transform.rotation.x = 0.0
    webcam_2_robot.transform.rotation.y = 0.0
    webcam_2_robot.transform.rotation.z = 0.0
    webcam_2_robot.transform.rotation.w = 1.0

    # Define a basic point that we can apply all the tfs to
    robot_pose = PoseStamped()
    robot_pose.header = Header()
    robot_pose.header.stamp = rospy.Time.now()
    robot_pose.pose = Pose()
    robot_pose.pose.position = Point(0.0, 0.0, 0.0)
    robot_pose.pose.orientation = Quaternion(0.0, 0.0, 0.0, 1.0)

    pose_pub = rospy.Publisher('/jetson/localizer_robot_pose', PoseStamped, queue_size=10)
    aruco_pose_pub = rospy.Publisher('/rviz/aruco_marker_pose', PoseStamped, queue_size=10)


    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        try:
            # Define tf between Aruco Marker and Webcam
            aruco_2_webcam_turned = tfBuffer.lookup_transform("aruco", "webcamTurned", rospy.Time())

            # Check if the new transform is the same as the last one
            if aruco_2_webcam_turned == last_transform:
                rate.sleep()
                continue

            last_transform = aruco_2_webcam_turned

            # Define tf between WebcamTurned and Webcam
            webcam_turned_2_webcam = TransformStamped()
            webcam_turned_2_webcam.header.frame_id = "webcamTurned"
            webcam_turned_2_webcam.child_frame_id = "webcam"
            webcam_turned_2_webcam.header.stamp = rospy.Time.now()
            webcam_turned_2_webcam.transform.translation.x = 0.0
            webcam_turned_2_webcam.transform.translation.y = 0.0
            webcam_turned_2_webcam.transform.translation.z = 0.0
            quat = tf.transformations.quaternion_from_euler(float(0.0),float(0.0),float(localizer_angle))
            webcam_turned_2_webcam.transform.rotation.x = quat[0]
            webcam_turned_2_webcam.transform.rotation.y = quat[1]
            webcam_turned_2_webcam.transform.rotation.z = quat[2]
            webcam_turned_2_webcam.transform.rotation.w = quat[3]

            # Sequentially apply the transforms to robot_pose in the following order
            world_2_webcam_turned = multiply_transforms(world_2_aruco, aruco_2_webcam_turned)
            world_2_webcam = multiply_transforms(world_2_webcam_turned, webcam_turned_2_webcam)
            
            rotation = [
                world_2_webcam.transform.rotation.x,
                world_2_webcam.transform.rotation.y,
                world_2_webcam.transform.rotation.z,
                world_2_webcam.transform.rotation.w
            ]
            eulers = tf.transformations.euler_from_quaternion(rotation)
            z_rotation = eulers[2]

            x_adj = l_magic * math.cos(z_rotation + theta_magic)
            y_adj = l_magic * math.sin(z_rotation + theta_magic)

            # Define tf between webcam and robot
            webcam_2_robot = TransformStamped()
            webcam_2_robot.header.frame_id = "webcam"
            webcam_2_robot.child_frame_id = "robot_unfused"
            webcam_2_robot.header.stamp = rospy.Time.now()
            webcam_2_robot.transform.translation.x = x_adj
            webcam_2_robot.transform.translation.y = y_adj
            webcam_2_robot.transform.translation.z = 0.0
            webcam_2_robot.transform.rotation.x = 0.0
            webcam_2_robot.transform.rotation.y = 0.0
            webcam_2_robot.transform.rotation.z = 0.0
            webcam_2_robot.transform.rotation.w = 1.0

            world_2_robot = multiply_transforms(world_2_webcam, webcam_2_robot)

            robot_pose_final = tf2_geometry_msgs.do_transform_pose(robot_pose, world_2_robot)

            # Publish the final Robot Pose
            pose_pub.publish(robot_pose_final)

            broadcaster.sendTransform(world_2_aruco)
            broadcaster.sendTransform(aruco_2_webcam_turned)
            broadcaster.sendTransform(webcam_turned_2_webcam)
            broadcaster.sendTransform(webcam_2_robot)
            broadcaster.sendTransform(world_2_robot)

            rate.sleep()

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue