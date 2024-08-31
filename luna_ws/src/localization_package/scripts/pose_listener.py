import rospy
from geometry_msgs.msg import PoseStamped

def callback(data):
    print(data.data)

def listener():

    rospy.init_node('pose_listener', anonymous=True)

    rospy.Subscriber('pose_data', PoseStamped, callback)

    rospy.spin()

if __name__ == '__main__':
    listener()
