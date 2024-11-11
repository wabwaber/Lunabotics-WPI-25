import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

def image_callback(image_msg):
    bridge = CvBridge()
    image = bridge.imgmsg_to_cv2(image_msg, desired_encoding="bgr8")
    cv2.imshow("Camera Image", image)
    cv2.waitKey(1)

def main():
    rospy.init_node('image_subscriber')
    rospy.Subscriber('camera_image_topic', Image, image_callback)
    rospy.spin()

if __name__ == '__main__':
    main()