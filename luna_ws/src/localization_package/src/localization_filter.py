import rospy
from std_msgs.msg import Float32MultiArray
import tf2_ros
import tf.transformations
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, TransformStamped
from std_msgs.msg import Header
import tf.transformations
import math

# Constants for filter tuning
alpha = 0.1  # Weight for localization estimates
beta = 0.9   # Weight for pose steps

pose = (0.0, 0.0, 0.0)
pose_step = (0.0, 0.0, 0.0)
localization_estimate = (0.0, 0.0, 0.0)
odom_timeout = 3.0
localizer_timeout = 10000.0 # TODO Change this line after testing
last_odom_time = rospy.Time(0)
last_localization_time = rospy.Time(0)


def update_odom_data_cb(odom_msg):
    global pose, pose_step, last_odom_time
    pose_step = odom_msg.data
    last_odom_time = rospy.Time.now()

    odom_pose = (
            (pose[0] + pose_step[0]*math.cos(pose[2]) - pose_step[1]*math.sin(pose[2])),
            (pose[1] + pose_step[0]*math.sin(pose[2]) + pose_step[1]*math.cos(pose[2])),
            (pose[2] + pose_step[2])
        )

    pose = odom_pose


def update_localization_estimate_cb(localization_estimate_msg):
    global pose, localization_estimate, last_localization_time

    x = localization_estimate_msg.pose.position.x
    y = localization_estimate_msg.pose.position.y
    quat = (
        localization_estimate_msg.pose.orientation.x,
        localization_estimate_msg.pose.orientation.y,
        localization_estimate_msg.pose.orientation.z,
        localization_estimate_msg.pose.orientation.w
    )
    euler = tf.transformations.euler_from_quaternion(quat)
    theta = euler[2]
    localization_estimate = [x, y, theta]
    last_localization_time = rospy.Time.now()

    delta_x = (localization_estimate[0] - pose[0]) * alpha
    delta_y = (localization_estimate[1] - pose[1]) * alpha

    delta_theta = localization_estimate[2] - pose[2]

    #constrain delta theta
    if(delta_theta > 3.14159):
        delta_theta -= 2 * 3.14159
    if(delta_theta < -3.14159):
        delta_theta += 2 * 3.14159
    
    delta_theta *= alpha

   
    # Async Complementary Filter Here
    fused_pose = (
        pose[0] + delta_x,
        pose[1] + delta_y,
        pose[2] + delta_theta
    )

    pose = fused_pose

def filter():
    
    global pose, last_odom_time, last_localization_time, localization_estimate

    rospy.init_node('localization_filter', anonymous=True)

    last_odom_time = rospy.Time.now()
    last_localization_time = rospy.Time.now()

    rospy.Subscriber('/jetson/pose_step', Float32MultiArray, update_odom_data_cb)

    rospy.Subscriber('/jetson/localizer_robot_pose', PoseStamped, update_localization_estimate_cb)

    filtered_pose_pub = rospy.Publisher('/jetson/filtered_pose', PoseStamped, queue_size=10)

    broadcaster = tf2_ros.StaticTransformBroadcaster()
    
    rate = rospy.Rate(10) # 10 hz

    while not rospy.is_shutdown():

        current_time = rospy.Time.now()

        # Check for timeouts
        if (current_time - last_odom_time).to_sec() > odom_timeout:
            rospy.logerr("Odom timeout occurred!")

        if (current_time - last_localization_time).to_sec() > localizer_timeout:
            rospy.logerr("Localization timeout occurred!")

        try:
            filtered_robot_pose = PoseStamped()
            filtered_robot_pose.header = Header()
            filtered_robot_pose.header.stamp = rospy.Time.now()
            filtered_robot_pose.pose = Pose()
            filtered_robot_pose.pose.position = Point(float(pose[0]), float(pose[1]), float(0.0))
            quat = Quaternion()
            quat_vals = tf.transformations.quaternion_from_euler(float(0.0), float(0.0), float(pose[2]))
            quat.x = quat_vals[0]
            quat.y = quat_vals[1]
            quat.z = quat_vals[2]
            quat.w = quat_vals[3]
            filtered_robot_pose.pose.orientation = quat
            filtered_pose_pub.publish(filtered_robot_pose)

            robot_transform = TransformStamped()
            robot_transform.header.frame_id = "world"
            robot_transform.child_frame_id = "robot"
            robot_transform.header.stamp = rospy.Time.now()
            robot_transform.transform.translation.x = float(pose[0])
            robot_transform.transform.translation.y = float(pose[1])
            robot_transform.transform.translation.z = float(0.0)
            robot_transform.transform.rotation.x = quat_vals[0]
            robot_transform.transform.rotation.y = quat_vals[1]
            robot_transform.transform.rotation.z = quat_vals[2]
            robot_transform.transform.rotation.w = quat_vals[3]

            broadcaster.sendTransform(robot_transform)

        except:
            print("There was an error in publishing localization filter data!")
        
        rate.sleep()

if __name__ == '__main__':
    filter()