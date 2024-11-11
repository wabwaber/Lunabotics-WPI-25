import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

def contour_image_callback(msg):
    bridge = CvBridge()
    contour_image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
    cv2.imshow("Contour Image", contour_image)
    cv2.waitKey(1)

def main():
    rospy.init_node('contour_image_subscriber', anonymous=True)
    rospy.Subscriber('/realsense/depth/contour_image', Image, contour_image_callback)
    rospy.spin()

if __name__ == '__main__':
    main()