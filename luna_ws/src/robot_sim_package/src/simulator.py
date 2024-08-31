from math import cos, sqrt, sin
import rospy
from std_msgs.msg import Float32MultiArray, Float32, Int32

rospy.init_node("simulator")

WHEEL_BASE_RADIUS = 0.3683
M_PER_TICK = 0.00015585
ROBOT_WIDTH_M = .5334
ROBOT_LENGTH_M = .5080

# state variables
global state
state = 0
global drive_speed
drive_speed = 0
global icc
icc = 0
prev_time = 0
pose_step_msg = Float32MultiArray()

# callback functions
def drive_cb(drive_speed_data):
    # convert drive speed from ticks/millis to meters/millis
    global drive_speed
    drive_speed = drive_speed_data.data * M_PER_TICK

def state_cb(state_data):
    global state
    state = state_data.data

def icc_cb(icc_data):
    global icc
    icc = icc_data.data

# publishers
step_pub = rospy.Publisher("/jetson/pose_step", Float32MultiArray, queue_size = 10)

# subscribe and hit that like button
rospy.Subscriber('/drivetrain/drive', Float32, drive_cb)
rospy.Subscriber('/drivetrain/state', Int32, state_cb)
rospy.Subscriber('/drivetrain/icc', Float32, icc_cb)

if __name__ == '__main__':
    prev_time = rospy.get_time() * 1000

    while not rospy.is_shutdown():
        new_time = rospy.get_time() * 1000
        delta_time = new_time - prev_time

        # diabled state
        if(state == 0):
            pose_step_msg.data = [0.0, 0.0, 0.0]

        # drive straight state
        elif (state == 1):
            pose_step_msg.data = [drive_speed * delta_time, 0.0, 0.0]

        # point turn state
        elif (state == 2):
            pose_step_msg.data = [0.0, 0.0, drive_speed * delta_time / WHEEL_BASE_RADIUS]

        # icc turning state
        elif (state == 3):
            if(icc > 0):
                max_displacement = drive_speed * delta_time
                r = sqrt((icc + ROBOT_WIDTH_M/2) ** 2 + (ROBOT_LENGTH_M) ** 2)
                theta = max_displacement / r

                pose_step_msg.data[0] = icc * sin(theta)
                pose_step_msg.data[1] = icc - icc * cos(theta)
                pose_step_msg.data[2] = theta

            elif(icc < 0):
                max_displacement = drive_speed * delta_time
                r = sqrt((icc - ROBOT_WIDTH_M/2) ** 2 + (ROBOT_LENGTH_M) ** 2)
                theta = -max_displacement / r

                pose_step_msg.data[0] = icc * sin(theta)
                pose_step_msg.data[1] = icc - icc * cos(theta)
                pose_step_msg.data[2] = theta
            
            elif(icc == 0):
                pose_step_msg.data = [0.0, 0.0, drive_speed * delta_time / WHEEL_BASE_RADIUS]

        else:
            print("I dont know where the fuck I am")
            pose_step_msg.data = [0.0, 0.0, 0.0]
        
        step_pub.publish(pose_step_msg)

        prev_time = new_time
        rospy.sleep(.1)
