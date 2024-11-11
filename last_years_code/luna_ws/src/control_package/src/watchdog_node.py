import rospy
from std_msgs.msg import Bool, Float32, Int32
import subprocess
import Jetson.GPIO as GPIO
from time import sleep

FAST_SPEED = 10

# Init last time received
last_time_received = 0
last_fast_speed = 0
recent_setpoint = 0
last_restart_time = 0
min_restart_interval = 15  # Minimum interval in seconds between restarts

init_en = False

# Setup GPIO
GPIO.cleanup()  # Reset all GPIO pins
GPIO.setmode(GPIO.BOARD)  # BOARD pin-numbering scheme
GPIO.setup(37, GPIO.OUT)  # Pin 37 as an output
GPIO.output(37, GPIO.LOW) # Initially set the pin to HIGH

def restart_node():
    global last_restart_time, last_fast_speed, last_time_received, init_en
    current_time = rospy.get_time()
    if current_time - last_restart_time < min_restart_interval:
        rospy.loginfo("Restart requested too soon after last restart.")
        return
    elif init_en:
        last_restart_time = current_time
        GPIO.output(37, GPIO.LOW)  # Set GPIO pin 37 low
        rospy.logerr("Restarting Power Systems!")
        subprocess.call(["rosnode", "kill", "/serial_node"])
        subprocess.Popen(["roslaunch", "robot_package", "serial_node.launch"])
        init_en = False
        for i in range(40):
            last_fast_speed = rospy.get_time() # Reset to prevent auto retriggers
            last_time_received = rospy.get_time() # Reset to prevent auto retriggers
            sleep(0.25)
        GPIO.output(37, GPIO.HIGH)  # Set back to high
        rospy.logerr("Done Resetting Power!")
    else:
        rospy.logerr("Can't reset if we haven't received a first good message yet.")

def watchdog_callback(data):
    global last_time_received, init_en
    if not init_en:
        GPIO.output(37, GPIO.HIGH) # Set the pin to HIGH after receiving first good message
        init_en = True
    last_time_received = rospy.get_time()
    rospy.loginfo("Data received from Arduino!")

def setpoint_callback(setpoint):
    global recent_setpoint
    recent_setpoint = setpoint.data

def speed_callback(speed):
    global last_fast_speed, recent_setpoint, last_time_received
    current_time = rospy.get_time()
    
    # If recent setpoint is close to 0, then set last fast speed to now
    if abs(recent_setpoint) < 1000:
        last_fast_speed = current_time
    elif abs(speed.data) > FAST_SPEED:
        last_fast_speed = current_time

    # If long time since last fast time, reset arduino
    if current_time - last_fast_speed > 3:
        rospy.logerr("Weird conveyor speed from arduino, restarting node!")
        restart_node()

def watchdog():
    global last_time_received, last_fast_speed
    rospy.init_node('rosserial_watchdog')
    rospy.Subscriber("/watchdog_bool", Bool, watchdog_callback)
    rospy.Subscriber("/digger/conveyor_current", Int32, setpoint_callback)
    rospy.Subscriber("/jetson/conveyor_speed", Float32, speed_callback)
    rate = rospy.Rate(3)  # check 3 times every second
    sleep(10) # allow for topics to set up
    last_time_received = rospy.get_time()

    while not rospy.is_shutdown():

        rospy.loginfo(rospy.get_time() - last_time_received)

        if rospy.get_time() - last_time_received > 1:  # 1 second without update
            rospy.logerr("No updates from rosserial_python, restarting node!")
            restart_node()
        rate.sleep()

if __name__ == '__main__':
    try:
        watchdog()
    except rospy.ROSInterruptException:
        pass
    finally:
        GPIO.output(37, GPIO.LOW)
        GPIO.cleanup()  # This ensures all GPIO resources are freed properly
