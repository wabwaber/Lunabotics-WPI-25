import rospy
from pynput import keyboard
from std_msgs.msg import Int32, Float32, Bool
import numpy as np

# Constants
DIGGER_UPDATE_HZ = 20
PLUNGE_BASE_EFFORT = 6
PLUNGE_KP = 12
ZEB_SPEED = 10.24
UNJAM_ERROR = 9
UNJAM_DURATION = 0.4
MIN_SPEED_READINGS = 50
MIN_PLUNGE_READINGS = 200
MAX_DRIVE_TIME = 100
DRIVE_SPEED = 0.15
MAX_REV_TIME = 0
REV_SPEED = 1.0
CONVEYOR_MAX_EFFORT = 9000
CONVEYOR_REVERSE_EFFORT = -11000
PLUNGER_RETRACT_SPEED = -35
PLUNGER_RETRACT_SPEED_FAST = -45
FAST_PLUNGER_SPEED = 28
FAST_PLUNGE_TIME = 12.0

class StateMachine:
    def __init__(self):
        self.states = ['plunging', 'retracting', 'all stop', 'keyboard']
        self.current_state = 2
        self.state_names = {
            'f': 0,
            'g': 1,
            'h': 2,
            'j': 3,
        }
        self.plunge_top = False
        self.plunge_bot = False
        self.prev_time = rospy.get_time()
        self.elapsed_drive_time = 0.0
        self.digging_started = False
        self.dig_start_time = 0.0

        self.listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        self.listener.start()

        # Create a loop to continuously print the current state at 20 Hz
        rospy.Timer(rospy.Duration(1.0 / DIGGER_UPDATE_HZ), self.loop)

        # Initialize publishers for conveyor and plunger
        self.run_conveyor_pub = rospy.Publisher('/digger/conveyor_current', Int32, queue_size=10)
        self.prev_conveyor_current = 0

        self.plunge_pub = rospy.Publisher('/digger/plunge', Int32, queue_size=10)
        self.prev_plunge_speed = 0

        self.drive_speed_pub = rospy.Publisher('/drivetrain/drive', Float32, queue_size=10)
        self.prev_drive_speed = 0.0

        self.drivetrain_state_pub = rospy.Publisher('/drivetrain/state', Int32, queue_size=10)
        self.prev_drive_state = 0

        self.dump_pub = rospy.Publisher('/deposit/open', Bool, queue_size=10)
        self.prev_open = False

        # Initialize subscriber for conveyor speed
        rospy.Subscriber('/jetson/conveyor_speed', Float32, self.conveyor_speed_cb)

        # Initialize subscriber for detecting when we are fully plunged and retracted
        rospy.Subscriber('/jetson/plunge_bot', Bool, self.plunge_bot_cb)
        rospy.Subscriber('/jetson/plunge_top', Bool, self.plunge_top_cb)

        # Initialize error variable
        self.error = 0

        # Initialize list for storing last 9 speed values
        self.speed_history = []

        # Jam Timer
        self.jam_time = None
        self.num_speed_readings = 0

    def on_press(self, key):
        try:
            key_char = key.char
            if key_char in self.state_names:
                new_state_index = self.state_names[key_char]
                if new_state_index != self.current_state:
                    self.current_state = new_state_index
                    self.num_speed_readings = 0
                    self.speed_history = []
            elif key_char == 'o':
                self.prev_open = not self.prev_open    
                self.dump_pub.publish(self.prev_open)
                rospy.logwarn("Toggling Deposit State!")
            elif self.current_state == 3:
                if key_char == 'w':
                    self.publish_new_drive(1.0)
                elif key_char == 's':
                    self.publish_new_drive(-1.0)
                elif key_char == '0':
                    self.publish_new_drive_state(0)
                    rospy.logwarn("Switching into DISABLED State!")
                elif key_char == '1':
                    self.publish_new_drive_state(1)
                    rospy.logwarn("Switching into DRIVING STRAIGHT State!")
                elif key_char == '2':
                    self.publish_new_drive_state(2)
                    rospy.logwarn("Switching into POINT TURNING State!")
                elif key_char == '3':
                    self.publish_new_drive_state(3)
                    rospy.logwarn("Switching into STRAFE State!")
                elif key_char == '4':
                    self.publish_new_drive_state(4)
                    rospy.logwarn("Switching into LEFT DRIVETRAIN RECOVERY Mode!")
                elif key_char == '5':
                    self.publish_new_drive_state(5)
                    rospy.logwarn("Switching into RIGHT DRIVETRAIN RECOVERY Mode!")
                else:
                    self.publish_new_drive(0.0)
        except AttributeError:
            pass
    def on_release(self, key):
        try:
            key_char = key.char
            if self.current_state == 3:
                if key_char == 'w':
                    self.publish_new_drive(0.0)
                elif key_char == 's':
                    self.publish_new_drive(0.0)
        except AttributeError:
            pass

    def publish_new_drive_state(self, new_drive_state_val):
        if new_drive_state_val != self.prev_drive_state:
            self.drivetrain_state_pub.publish(new_drive_state_val)
            self.prev_drive_state = new_drive_state_val

    def publish_new_drive(self, new_drive_val):
        if new_drive_val != self.prev_drive_speed:
            self.drive_speed_pub.publish(new_drive_val)
            self.prev_drive_speed = new_drive_val
            rospy.logwarn(f"Publishing a drive speed of {new_drive_val}")

    def publish_new_conveyor(self, new_conveyor_val):
        if new_conveyor_val != self.prev_conveyor_current:
            self.run_conveyor_pub.publish(new_conveyor_val)
            self.prev_conveyor_current = new_conveyor_val
            rospy.logwarn(f"Publishing a conveyor speed of {new_conveyor_val}")

    def publish_new_plunge(self, new_plunge_val):
        if new_plunge_val != self.prev_plunge_speed:
            self.plunge_pub.publish(new_plunge_val)
            self.prev_plunge_speed = new_plunge_val
            rospy.logwarn(f"Publishing a plunge speed of {new_plunge_val}")

    def loop(self, event):

        # Get current ROS time
        current_time = rospy.get_time()

        # Check for All Stop case first for safety reasons
        if self.current_state == 2:

            # Set Digging Started to FALSE
            self.digging_started = False
            rospy.loginfo("Resetting Digging Started...")

            rospy.loginfo("ALL STOP")

            # Stop Conveyor and Plunger and Drivetrain
            self.publish_new_conveyor(0)
            self.publish_new_plunge(0)
            self.publish_new_drive_state(0)
            self.publish_new_drive(0)

        elif self.current_state == 3:

            # Set Digging Started to FALSE
            self.digging_started = False
            rospy.loginfo("Resetting Digging Started...")

            rospy.loginfo("DRIVETRAIN CTRL")

            # Stop Conveyor and Plunger
            self.publish_new_conveyor(0)
            self.publish_new_plunge(0)
        
        else:

            # Block other stuff from happening
            if self.jam_time is not None and (current_time - self.jam_time) < UNJAM_DURATION:
                self.num_speed_readings = 0
                self.error = 0
                rospy.logwarn("JAMMED! UNJAMMING...")
                return

            if self.current_state == 0:  # If state is plunging

                self.publish_new_drive_state(1)

                # Drivetrain logic
                if self.plunge_bot:

                    if self.prev_time is not None:
                        self.elapsed_drive_time += current_time - self.prev_time

                    # Check to make sure max drive time is not reached
                    if self.elapsed_drive_time > MAX_DRIVE_TIME:

                        rospy.loginfo("Done driving forward!")

                        # Stop Drivetrain and transition to retracting
                        self.publish_new_drive(0.0)
                        self.publish_new_drive_state(0)
                        self.elapsed_drive_time = 0.0
                        self.current_state = 1
                    else:
                        # Drive Drivetrain
                        self.publish_new_drive_state(1)
                        self.publish_new_drive(DRIVE_SPEED)
                        rospy.loginfo("Driving forwards!")
                        rospy.loginfo(self.elapsed_drive_time)
                        
                else:
                    # Stop Drivetrain
                    self.publish_new_drive(0.0)
                    self.publish_new_drive_state(0)
                    rospy.loginfo("Plunging!")

                # Conveyor publish set effort and monitor speed
                self.publish_new_conveyor(CONVEYOR_MAX_EFFORT)

                # Get current ROS time
                current_time = rospy.get_time()

                # If its the start of digging, plunge fast
                if current_time - self.dig_start_time < FAST_PLUNGE_TIME:
                    plunger_speed = FAST_PLUNGER_SPEED
                    rospy.loginfo(f"PLUNGING FAST {int((current_time - self.dig_start_time) / FAST_PLUNGE_TIME * 100)}% COMPLETE")
                else:
                    # Plunger publish plunge speed as a function of conveyor speed
                    plunger_speed = self.calculate_plunge_speed(PLUNGE_BASE_EFFORT)

                # If conveyor has had time to speed up or detects further jam
                if self.num_speed_readings > MIN_PLUNGE_READINGS:

                    # If Digging is not started, make sure to record the start time
                    if self.digging_started == False and self.plunge_top:
                        self.dig_start_time = rospy.get_time()
                        self.digging_started = True

                    # Run Plunger
                    self.publish_new_plunge(int(plunger_speed))
                else:
                    
                    # Retract Plunger and Stop Drivetrain
                    self.publish_new_plunge(-35)
                    self.publish_new_drive(0.0)

                # Unjamming logic
                if self.error > UNJAM_ERROR and self.num_speed_readings > MIN_SPEED_READINGS:
                    # Stop Drivetrain
                    self.publish_new_drive(0.0)

                    # Reverse Conveyor
                    self.publish_new_conveyor(CONVEYOR_REVERSE_EFFORT)
                    
                    # Retract Plunger
                    self.publish_new_plunge(-15)

                    self.num_speed_readings = 0
                    self.jam_time = current_time  # Record the jam start time

            elif self.current_state == 1:  # Else if state is retracting

                # Set Digging Started to FALSE
                self.digging_started = False
                rospy.loginfo("Resetting Digging Started...")

                self.publish_new_drive_state(1)

                # Drivetrain logic
                if self.plunge_top:

                    if self.prev_time is not None:
                        self.elapsed_drive_time += current_time - self.prev_time

                    # Check to make sure max drive time is not reached
                    if self.elapsed_drive_time > MAX_REV_TIME:

                        rospy.loginfo("Done driving backwards!")

                        # Stop Drivetrain
                        self.publish_new_drive(0.0)

                    else:
                        self.publish_new_drive(-REV_SPEED)
                        rospy.loginfo("Driving backwards!")
                        rospy.loginfo(self.elapsed_drive_time)

                else:
                    # Stop Drivetrain
                    self.publish_new_drive(0.0)
                    rospy.loginfo("Retracting!!")

                # Stop Conveyor
                self.publish_new_conveyor(0)

                # Plunger publish constant retract plunge speed
                self.publish_new_plunge(-45)
                self.jam_time = None

        self.prev_time = current_time

    def calculate_plunge_speed(self, base):
        base = int(base)
        speed = int(base - (PLUNGE_KP * self.error))
        # Ensure that the plunger doesn't go faster than base speed
        if speed > base:
            return base
        # Ensure that the plunger doesn't go faster than negative half base speed
        elif speed < (-base / 2):
            return -base / 2
        return speed

    def conveyor_speed_cb(self, msg):

        self.speed_history.append(msg.data)
        self.num_speed_readings += 1
        
        # If the length of the speed history list exceeds 9, remove the oldest value
        if len(self.speed_history) > 9:
            del self.speed_history[0]
        
        # Apply median filter to the speed history list
        median_speed = np.median(self.speed_history)
        
        # Update error using the median speed value
        self.error = ZEB_SPEED - median_speed

    def plunge_top_cb(self, msg):
        self.plunge_top = msg.data

    def plunge_bot_cb(self, msg):
        self.plunge_bot = msg.data

if __name__ == '__main__':
    try:
        rospy.init_node('state_machine')
        state_machine = StateMachine()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
