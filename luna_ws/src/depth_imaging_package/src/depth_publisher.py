import rospy
import cv2
import numpy as np
import pyrealsense2 as rs
from collections import deque
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Float32MultiArray

# Set up a queue for temporal smoothing
depth_buffer = deque(maxlen=5)

min_depth_difference = 10  # Minimum depth difference to consider as an object
canny_threshold1 = 10 # Was previously 30
canny_threshold2 = 50 # Was previously 100

# Convex Hull Filtering Criteria
min_size = 4000
max_size = 27000
min_roundness = 0.7

SHOW_CONTOUR = False
SHOW_CONVEX = False

IMAGE_WIDTH = 640
IMAGE_HEIGHT = 480

WIDTH_CROP = 0.05
HEIGHT_CROP = 0.3

def GetBGR(frame_color):
    # Input: Intel handle to 16-bit YU/YV data
    # Output: BGR8
    H = frame_color.get_height()
    W = frame_color.get_width()
    Y = np.frombuffer(frame_color.get_data(), dtype=np.uint8)[0::2].reshape(H,W)
    UV = np.frombuffer(frame_color.get_data(), dtype=np.uint8)[1::2].reshape(H,W)
    YUV =  np.zeros((H,W,2), 'uint8')
    YUV[:,:,0] = Y
    YUV[:,:,1] = UV
    BGR = cv2.cvtColor(YUV,cv2.COLOR_YUV2BGR_YUYV)
    return BGR

def main():
    rospy.init_node('realsense_depth_publisher', anonymous=True)

    try: 
        # Initialize RealSense pipeline
        pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, IMAGE_WIDTH, IMAGE_HEIGHT, rs.format.z16, 15)
        config.enable_stream(rs.stream.color, IMAGE_WIDTH, IMAGE_HEIGHT, rs.format.yuyv, 15)

        # Start streaming
        pipeline.start(config)

        # Set up align object
        align = rs.align(rs.stream.color)

        # Set up ROS publisher
        # image_pub = rospy.Publisher('/realsense/depth/image_aligned', Image, queue_size=10)
        contour_pub = rospy.Publisher('/realsense/depth/contour_image', Image, queue_size=10)
        obstacle_pub = rospy.Publisher('/realsense/depth/obstacle', Float32MultiArray, queue_size=10)
        bridge = CvBridge()

        rate = rospy.Rate(3)  # 1 Hz

        try:
            while not rospy.is_shutdown():
                # Wait for the next set of frames
                frames = pipeline.wait_for_frames()

                # Align the depth frame to the color frame
                aligned_frames = align.process(frames)
                depth_frame = aligned_frames.get_depth_frame()
                color_frame = aligned_frames.get_color_frame()

                if not depth_frame or not color_frame:
                    continue

                # Convert from YUYV to BGR
                color_frame_np = GetBGR(color_frame)

                # Convert depth frame to depth image
                depth_data = np.asanyarray(depth_frame.get_data())

                # Crop Images to reduce processing time
                height, width, _ = color_frame_np.shape
                crop_height = int(height * HEIGHT_CROP)
                crop_width = int(width * WIDTH_CROP)
                color_frame_np = color_frame_np[0:-crop_height, crop_width:-crop_width, :]
                depth_data = depth_data[0:-crop_height, crop_width:-crop_width]

                # Apply spatial smoothing (Gaussian blur)
                smoothed_depth_data = cv2.GaussianBlur(depth_data, (3, 3), 0)

                # Convert to 8-bit unsigned integer
                smoothed_depth_data = (smoothed_depth_data / np.max(smoothed_depth_data) * 255).astype(np.uint8)

                # Compute the gradient using the Canny edge detector
                edges = cv2.Canny(smoothed_depth_data, canny_threshold1, canny_threshold2)

                # Apply morphological operations
                kernel = np.ones((7, 7), np.uint8)
                edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel)

                # Find contours in the edges
                contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

                # Filter contours based on depth difference
                for contour in contours:
                    # Calculate the average depth within the contour
                    mask = np.zeros_like(smoothed_depth_data, dtype=np.uint8)
                    cv2.drawContours(mask, [contour], 0, 255, thickness=cv2.FILLED)
                    avg_depth = np.mean(smoothed_depth_data[mask > 0])

                    # Check if the depth difference is significant
                    if np.abs(avg_depth - smoothed_depth_data[contour[0][0][1], contour[0][0][0]]) > min_depth_difference:

                        if SHOW_CONTOUR:
                            # Draw the contour on the color frame
                            cv2.drawContours(color_frame_np, [contour], 0, (0, 255, 0), 2)

                        else:
                            # TODO Delete this line
                            cv2.drawContours(color_frame_np, [contour], 0, (0, 255, 0), 2)      

                            # Find the convex hull of the contour
                            convex_hull = cv2.convexHull(contour)

                            area_convex_hull = cv2.contourArea(convex_hull)
                            perimeter_convex_hull = cv2.arcLength(convex_hull, True)
                            roundness = (4 * np.pi * area_convex_hull) / (perimeter_convex_hull ** 2)

                            if SHOW_CONVEX:
                                cv2.drawContours(color_frame_np, [convex_hull], 0, (255, 0, 0), 2)
                                print(f"Convex Hull Size: {cv2.contourArea(convex_hull)}")
                                print(f"Roundness: {roundness}")
                                cv2.imshow("Convex Hulls", color_frame_np)
                                cv2.waitKey(0)

                            else:
                                # Filter convex hulls based on size and roundness
                                if min_size <= area_convex_hull <= max_size and roundness >= min_roundness:

                                    # Draw the convex hull on the color frame
                                    cv2.drawContours(color_frame_np, [convex_hull], 0, (255, 0, 0), 2)

                                    # Draw the smallest enclosing circle
                                    (center, radius) = cv2.minEnclosingCircle(convex_hull)
                                    center = tuple(map(int, center))
                                    radius = int(radius)
                                    cv2.circle(color_frame_np, center, radius, (0, 0, 255), 2)

                                    # Find cartesian coords of center of object
                                    u, v = center[1], center[0]
                                    depth = depth_data[u][v] / 1000 # Depth in meters

                                    # Get X, Y, Z coordinates
                                    depth_point = rs.rs2_deproject_pixel_to_point(depth_frame.profile.as_video_stream_profile().intrinsics, [v, u], depth)
                                    x0, y0, z0 = depth_point
                                    x0, y0, z0 = map(lambda val: round(val, 2), (x0, y0, z0))

                                    # Find radius in meters for object mapping
                                    if (center[0] + radius < 576):
                                        s, t = center[1], (center[0] + radius)
                                    elif (center[0] - radius > 0):
                                        s, t = center[1], (center[0] - radius)
                                    else:
                                        s, t = u, v
                                    depth = depth_data[s][t] / 1000 # Depth in meters

                                    # Get X, Y, Z coordinates
                                    depth_point = rs.rs2_deproject_pixel_to_point(depth_frame.profile.as_video_stream_profile().intrinsics, [t, s], depth)
                                    x1, y1, z1 = depth_point
                                    x1, y1, z1 = map(lambda val: round(val, 2), (x1, y1, z1))

                                    rad_m = abs(x0 - x1)

                                    # Publish Obstacle pose in Realsense Camera Frame
                                    obstacle_msg = Float32MultiArray()
                                    obstacle_msg.data = [x0, y0, z0, rad_m]
                                    obstacle_pub.publish(obstacle_msg)

                # cv2.imshow("Color Frame", color_frame_np)
                # cv2.waitKey(1)

                # # Apply temporal smoothing
                # depth_buffer.append(depth_data)
                # smoothed_depth_data = np.mean(depth_buffer, axis=0)

                # Publish smoothed depth image to ROS
                # depth_ros_msg = bridge.cv2_to_imgmsg(smoothed_depth_data, encoding="passthrough")
                # depth_ros_msg.header.stamp = rospy.Time.now()
                # image_pub.publish(depth_ros_msg)

                # Publish contour BGR image to ROS
                scaled_image = cv2.resize(color_frame_np, (0, 0), fx=0.5, fy=0.5)
                contour_ros_msg = bridge.cv2_to_imgmsg(scaled_image, encoding="bgr8")
                contour_ros_msg.header.stamp = rospy.Time.now()
                contour_pub.publish(contour_ros_msg)

                rate.sleep()

        except rospy.ROSInterruptException:
            pass

        finally:
            # Stop streaming
            pipeline.stop()

    except Exception as e:
        print(e)
        pass

if __name__ == '__main__':
    main()