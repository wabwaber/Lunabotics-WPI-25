import rospy
from std_msgs.msg import Int32
import Jetson.GPIO as GPIO
import time

class ArduinoWatchdog:
    def __init__(self):
        rospy.init_node('arduino_watchdog', anonymous=True)

        # Set up GPIO pin
        self.reset_pin = 18
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.reset_pin, GPIO.OUT)

        # Initialize timer variables
        self.last_message_time = time.time()

        # Subscribe to the Arduino topic
        rospy.Subscriber("/arduino_watchdog", Int32, self.watchdog_cb)

    def watchdog_cb(self, data):
        # Update the last message time
        self.last_message_time = time.time()

    def run(self):
        rate = rospy.Rate(1)  # 1 Hz 
        while not rospy.is_shutdown():
            # Check if timeout has occurred
            if time.time() - self.last_message_time > 5:
                # Reset Arduino by setting GPIO pin LOW
                GPIO.output(self.reset_pin, GPIO.LOW)
                rospy.loginfo("Arduino reset triggered")
                time.sleep(0.5)  # Ensure reset signal is sent
                # Set GPIO pin back to HIGH
                GPIO.output(self.reset_pin, GPIO.HIGH)

                # TODO Delete after testing
                self.last_message_time = time.time() + 10

            rate.sleep()

if __name__ == '__main__':
    try:
        watchdog_node = ArduinoWatchdog()
        watchdog_node.run()
    except rospy.ROSInterruptException:
        pass
