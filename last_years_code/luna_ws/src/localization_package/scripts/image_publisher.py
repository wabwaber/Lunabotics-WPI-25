# Import necessary libraries
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2 as cv
import numpy as np
from math import sin, cos
import cv2.aruco as aruco
from std_msgs.msg import Float32, Float32MultiArray
import pickle
from scipy.spatial.transform import Rotation as R
import tf.transformations, tf2_ros, geometry_msgs.msg
import math

# Load the pickles
matrix_path = '/home/jetson/Development/Luna/luna_ws/src/localization_package/scripts/pkls027/cameraMatrix.pkl'
dist_path = '/home/jetson/Development/Luna/luna_ws/src/localization_package/scripts/pkls027/dist.pkl'

with open(matrix_path, 'rb') as file:
    camera_matrix = pickle.load(file)

with open(dist_path, 'rb') as file:
    dist = pickle.load(file)

# Define ArUco Characteristics in Centimeters
LARGE_MARKER_SIZE = 88
SMALL_MARKER_SIZE = 12.6

# Load the ArUco dictionary
dictionary = aruco.getPredefinedDictionary(cv.aruco.DICT_5X5_1000)

# Averaging Filter
xws = []
yws = []
AVERAGING_FILTER_SIZE = 30

ARUCO_TARGET_THRESHOLD = 80

# Angle of the localizer turret, used in the transformation matrix to go from webcam to robot pose
localizer_angle = 0.0

# Function to clean corners
def clean_corners(corners):
    clean_corners = []
    corners = (corners[0][0][0], corners[0][0][1], corners[0][0][2], corners[0][0][3])
    for corner in corners:
        corner = [corner[0], corner[1]]
        clean_corners.append(corner)
    return clean_corners

# Function to calculate the centroid of 4 points
def calculate_centroid(points):
    if len(points) != 4:
        raise ValueError("Input list must contain exactly 4 (x, y) pairs.")
    
    # Calculate the sum of x and y coordinates
    sum_x = sum(point[0] for point in points)
    sum_y = sum(point[1] for point in points)
    
    # Calculate the centroid coordinates
    centroid_x = int(sum_x / 4)
    centroid_y = int(sum_y / 4)
    
    return (centroid_x, centroid_y)


def main():
    global xws, yws
    rospy.init_node('image_publisher')
    image_publisher = rospy.Publisher('camera_image_topic', Image, queue_size=10)
    servo_error_publisher = rospy.Publisher('/localizer/raw_error', Float32, queue_size=10)
    tvec_publisher = rospy.Publisher('/tvec', Float32MultiArray, queue_size=10)
    aruco_broadcaster = tf2_ros.StaticTransformBroadcaster()

    bridge = CvBridge()

    # Falied Readings
    failed_readings = 0
    MAX_ALLOWED_FAILED_READINGS = 10

    # Define Camera to Use
    cam = cv.VideoCapture(0)

    # Define green color
    YELLOW = (0, 255, 255)

    rate = rospy.Rate(10)  # Publish at 10 Hz

    while not rospy.is_shutdown():
        # Capture an image from the camera
        ret, frame = cam.read()

        if ret:

            if failed_readings > MAX_ALLOWED_FAILED_READINGS:

                # Empty Avg Xs and Ys
                xws = []
                yws = []

            # BGR 2 Gray
            gray_frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

            # Blur to remove noise
            gray_frame = cv.blur(gray_frame, (5,5))

            frame = cv.cvtColor(gray_frame, cv.COLOR_GRAY2BGR)

            # Detect ArUco markers in the frame.
            marker_corners, ids, _ = aruco.detectMarkers(gray_frame, dictionary)
            height, width, channels = frame.shape

            # Draw detected markers on the frame.
            if ids is not None:

                # Find the index of the marker with ID 256
                index_256 = np.where(ids == 256)[0]

                if len(index_256) > 0:
                    i = index_256[0]  # Use the first occurrence of marker with ID 256
                    marker_size = LARGE_MARKER_SIZE # Set Marker Size to Large

                else:
                    # Marker with ID 256 not found, check for marker with ID 395
                    index_395 = np.where(ids == 395)[0]

                    if len(index_395) > 0:
                        i = index_395[0]  # Use the first occurrence of marker with ID 395
                        marker_size = SMALL_MARKER_SIZE # Set Marker Size to Small

                    else:
                        # Neither marker with ID 256 nor ID 395 found, continue to the next iteration
                        continue


                # Calculate Rotation and Translation for the selected marker
                rVec, tVec, _ = aruco.estimatePoseSingleMarkers(
                    [marker_corners[i]], marker_size, camera_matrix, dist
                )

                cv.polylines(
                    frame, [marker_corners[i].astype(np.int32)], True, YELLOW, 4, cv.LINE_AA
                )

                corners = marker_corners[i].reshape(4, 2)
                corners = corners.astype(int)

                centroid = calculate_centroid(corners)
                servo_error = int((width//2) - centroid[0])
                servo_error_publisher.publish(servo_error)

                if abs(servo_error) < ARUCO_TARGET_THRESHOLD:
                    # Convert rotation vector to rotation matrix using Rodrigues' rotation formula
                    rotation_matrix, _ = cv.Rodrigues(rVec)

                    # Extract the yaw from the rotation matrix
                    theta = np.arctan2(-rotation_matrix[2, 0], np.sqrt(rotation_matrix[2, 1]**2 + rotation_matrix[2, 2]**2))

                    # Extract x, y, and z from the translation vector
                    x, y, z = tVec[0][0][0], tVec[0][0][1], tVec[0][0][2]
                    tvec_msg = Float32MultiArray()
                    tvec_msg.data = [x, y, z]
                    tvec_publisher.publish(tvec_msg)

                    xw = round((z * cos(theta) - x * sin(theta)), 1)
                    yw = round((x * cos(theta) - z * sin(theta)), 1) * -1

                    if len(xws) >= AVERAGING_FILTER_SIZE:
                        xws.pop(0)

                    if len(yws) >= AVERAGING_FILTER_SIZE:
                        yws.pop(0)

                    xws.append(xw)
                    yws.append(yw)

                    # Take the avg, round, and convert to meters
                    avg_xw = round(sum(xws) / len(xws), 1) / 100
                    avg_yw = round(sum(yws) / len(yws), 1) / 100

                    # Publish as a transformStamped msg
                    static_transformStamped = geometry_msgs.msg.TransformStamped()
                    static_transformStamped.header.stamp = rospy.Time.now()
                    static_transformStamped.header.frame_id = "aruco"
                    static_transformStamped.child_frame_id = "webcamTurned"
                    static_transformStamped.transform.translation.x = float(avg_xw)
                    static_transformStamped.transform.translation.y = float(avg_yw)
                    static_transformStamped.transform.translation.z = float(0.0)
                    quat = tf.transformations.quaternion_from_euler(float(0.0),float(0.0),float(3.1415 + theta))
                    static_transformStamped.transform.rotation.x = quat[0]
                    static_transformStamped.transform.rotation.y = quat[1]
                    static_transformStamped.transform.rotation.z = quat[2]
                    static_transformStamped.transform.rotation.w = quat[3]

                    aruco_broadcaster.sendTransform(static_transformStamped)

                    # Define the starting and ending points of the line
                    start_point = (frame.shape[1] // 2, 0)
                    end_point = (frame.shape[1] // 2, frame.shape[0])
                    RED = (0, 0, 255)
                    cv.line(frame, start_point, end_point, RED, 1)

                    failed_readings = 0

            else: 
                servo_error_publisher.publish(0.0)

            # Convert the OpenCV image to a ROS Image message
            scaled_image = cv.resize(frame, (0, 0), fx=0.25, fy=0.25)
            image_message = bridge.cv2_to_imgmsg(scaled_image, "bgr8")
            image_publisher.publish(image_message)

        failed_readings += 1
        rate.sleep()

    cam.release()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
