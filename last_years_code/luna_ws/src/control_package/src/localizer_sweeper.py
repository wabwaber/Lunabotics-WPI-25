import rospy
from std_msgs.msg import Float32, Bool


# Robot variables
localizer_error = 0.0
localizer_enable = False


def update_localizer_error_cb(localizer_error_msg):
    global localizer_error
    localizer_error = localizer_error_msg.data

def update_localizer_enable_cb(localizer_enable_msg):
    global localizer_enable
    localizer_enable = localizer_enable_msg.data

def main():
    global localizer_error, localizer_enable, drivetrain_enable, localizer_angle
    rospy.init_node('localizer_sweeper', anonymous=True)

    # Define publishers for different key presses
    localizer_error_pub = rospy.Publisher('/localizer/error', Float32, queue_size=10)

    # Subscribers for Autonomous Routine
    rospy.Subscriber('/localizer/raw_error', Float32, update_localizer_error_cb)
    rospy.Subscriber('/localizer/enable', Bool, update_localizer_enable_cb)

    iterator = 0

    rate = rospy.Rate(5)

    while not rospy.is_shutdown():

        # Autonomous Code Here

        # Only if the localizer is enabled
        if (localizer_enable):

            iterator += 1

            # Use localizer to search for the marker
            if (localizer_error == 0.0):
                if(iterator < 5):
                    localizer_error = 1000.0
                else:
                    iterator = 0
            localizer_error_pub.publish(localizer_error)
        else: 
            localizer_error_pub.publish(0.0)

        rate.sleep()

if __name__ == '__main__':
    try:
        main()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass