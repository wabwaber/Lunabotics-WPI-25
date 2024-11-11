import rospy
from std_msgs.msg import String


def ianoutputcallback(data):
     print(data.data)

def listener():

    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("/listener/ian_output", String, ianoutputcallback)

    rospy.spin()

if __name__ == '__main__':
    listener()