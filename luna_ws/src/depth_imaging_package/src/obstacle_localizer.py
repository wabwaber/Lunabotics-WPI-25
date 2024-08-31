import rospy
from std_msgs.msg import Float32MultiArray, Float32
from geometry_msgs.msg import PoseStamped, Pose, Point
import tf.transformations
import numpy as np
from math import cos, sin, sqrt, pi

# Define global variables

REALSENSE_OFFSET_X = 0.125
REALSENSE_OFFSET_Y = 0.215
REALSENSE_OFFSET_Z = 1.08

CORRECTION_FACTOR_X = -0.20
CORRECTION_FACTOR_Y = -0.155

robot_pose = [0, 0, 0]
obstacle_location_realsense = [0, 0, 0, 0]

def update_robot_pose(data):
    global robot_pose
    pose_x = data.pose.position.x
    pose_y = data.pose.position.y
    quat = data.pose.orientation
    roll, pitch, yaw = tf.transformations.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
    robot_pose = [pose_x, pose_y, yaw]

# Define callback functions
def update_obstacle_pose(data):
    global robot_pose, obstacle_location_realsense, REALSENSE_OFFSET_X, REALSENSE_OFFSET_Y, REALSENSE_OFFSET_Z, CORRECTION_FACTOR_X, CORRECTION_FACTOR_Y
    obstacle_location_realsense = data.data
    rad_m = obstacle_location_realsense[3]

    if robot_pose[0] != 0.0 and robot_pose[1] != 0.0:

        point_realsense = np.array([obstacle_location_realsense[2], -obstacle_location_realsense[0], -obstacle_location_realsense[1], 1])

        realsense_to_robot_tf = np.array([
            [0.707,     0,      0.707,      REALSENSE_OFFSET_X],
            [0,         1,      0,          REALSENSE_OFFSET_Y],
            [-0.707,    0,      0.707,      REALSENSE_OFFSET_Z],
            [0,         0,      0,          1]
        ])

        point_robot = np.dot(realsense_to_robot_tf, point_realsense)

        point_robot[0] += CORRECTION_FACTOR_X
        point_robot[1] += CORRECTION_FACTOR_Y

        robot_to_world_tf = np.array([
            [cos(robot_pose[2]),        -sin(robot_pose[2]),    0,          robot_pose[0]],
            [sin(robot_pose[2]),        cos(robot_pose[2]),     0,          robot_pose[1]],
            [0,                         0,                      1,          0],
            [0,                         0,                      0,          1]
        ])

        point_world = np.dot(robot_to_world_tf, point_robot)

        # print("X: " + str(robot_pose[0]) + "        Y: " + str(robot_pose[1]) + "       Yaw: " + str(robot_pose[2]))
        # print("Point X: " + str(point_world[0]) + "        Point Y: " + str(point_world[1]))

        obstacle = Float32MultiArray()
        obstacle.data = [point_world[0], point_world[1], rad_m]

        obstacle_pub.publish(obstacle)

# Initialize ROS node
rospy.init_node('obstacle_localizer', anonymous=True)

# Initialize publishers and subscribers
obstacle_pub = rospy.Publisher('/map/obstacle', Float32MultiArray, queue_size=10)
rospy.Subscriber('/realsense/depth/obstacle', Float32MultiArray, update_obstacle_pose)
rospy.Subscriber('/jetson/filtered_pose', PoseStamped, update_robot_pose)

# Define timer callback function
def timer_callback(event):
    pass  # Do nothing here as publishing is handled in the update_obstacle_pose callback

# Create a timer with a callback function that triggers publishing
timer = rospy.Timer(rospy.Duration(0.1), timer_callback)

# Spin ROS node
rospy.spin()