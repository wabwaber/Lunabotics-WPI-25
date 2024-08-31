import rospy
from std_msgs.msg import Float32, Bool, Int32
from pynput import keyboard

class KeyControlNode:
    def __init__(self):
        rospy.init_node('keyboard_control', anonymous=True)

        self.released = True

        self.conveyer_running = False

        # Define publishers for different key presses
        self.drivetrain_drive_pub = rospy.Publisher('/drivetrain/drive', Float32, queue_size=10)
        self.drive_speed = Float32()
        self.prev_drive_speed = 0.0

        self.drivetrain_state_pub = rospy.Publisher('/drivetrain/state', Int32, queue_size=10)
        self.drive_state = Int32()
        self.prev_drive_state = 0

        self.drivetrain_icc_pub = rospy.Publisher('/drivetrain/icc', Float32, queue_size=10)
        self.icc = Float32()
        self.prev_icc = 0

        self.localizer_error_pub = rospy.Publisher('localizer/error', Float32, queue_size=10)
        self.localizer_error = Float32()
        self.prev_localizer_error = 0.0

        self.localizer_enable_pub = rospy.Publisher('/localizer/enable', Bool, queue_size=10)
        self.localizer_enable = Bool()
        self.prev_localizer_enable = False

        self.run_conveyor_pub = rospy.Publisher('/digger/conveyor_current', Int32, queue_size=10)
        self.run_conveyor = Int32()
        self.prev_run_conveyor = 0

        self.plunge_pub = rospy.Publisher('/digger/plunge', Int32, queue_size=10)
        self.plunge_speed = Int32()
        self.prev_plunge_speed = 0

        self.dump_pub = rospy.Publisher('/deposit/open', Bool, queue_size=10)
        self.deposit_open = Bool()
        self.prev_open = False
        self.open = False

        # Create a listener for keyboard events
        self.listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        self.listener.start()

        # Initialize variables to track key states
        self.key_states = {
            'w': False,
            's': False,
            'q': False,
            'e': False,
            'n': False,
            'm': False,
            'b': False,
            '0': False,
            '1': False,
            '2': False,
            '3': False,
            'z': False,
            'x': False,
            'v': False,
            'a': False,
            'd': False,
            'o': False,
        }

        # Create a timer to check key presses periodically
        self.timer = rospy.Timer(rospy.Duration(0.1), self.check_key_presses)

    def on_press(self, key):
        try:
            key_char = key.char
            if key_char in self.key_states:
                self.key_states[key_char] = True
        except AttributeError:
            pass

    def on_release(self, key):
        try:
            self.released = True
            key_char = key.char
            if key_char in self.key_states:
                self.key_states[key_char] = False
        except AttributeError:
            pass

    def check_key_presses(self, event):
        # Keys needed for driving forward and backward
        if self.key_states['w']:
            self.drive_speed.data = 1.0
        elif self.key_states['s']:
            self.drive_speed.data = -1.0
        else:
            self.drive_speed.data = 0.0
        
        if self.drive_speed.data != self.prev_drive_speed:
            self.prev_drive_speed = self.drive_speed.data
            self.drivetrain_drive_pub.publish(self.drive_speed)
            
        # Keys needed for moving the ICC
        if self.key_states['q']:
            self.icc.data += .02
        elif self.key_states['e']:
            self.icc.data -= .02
        
        if self.icc.data != self.prev_icc:
            self.prev_icc = self.icc.data
            self.drivetrain_icc_pub.publish(self.icc)

        # Keys needed for the primary drie state machine

        if self.key_states['0']:
            self.drive_state.data = 0
            self.localizer_enable.data = False
        elif self.key_states['1']:
            self.drive_state.data = 1
            self.localizer_enable.data = True
        elif self.key_states['2']:
            self.drive_state.data = 2
            self.localizer_enable.data = True
        elif self.key_states['3']:
            self.drive_state.data = 3
            self.localizer_enable.data = True

        if self.drive_state.data != self.prev_drive_state:
            self.drivetrain_state_pub.publish(self.drive_state)
            self.prev_drive_state = self.drive_state.data
        if self.localizer_enable.data != self.prev_localizer_enable:
            self.prev_localizer_enable = self.localizer_enable.data
            self.localizer_enable_pub.publish(self.localizer_enable)

        if self.key_states['b']:
            self.localizer_error.data = 100.0
        elif self.key_states['m']:
            self.localizer_error.data = -100.0
        else:
            self.localizer_error.data = 0.0
        
        if self.localizer_error.data != self.prev_localizer_error:
            self.localizer_error_pub.publish(self.localizer_error)
            self.prev_localizer_error = self.localizer_error.data


        # Conveyor Spinny Stuff
        if self.key_states['z']:
            self.run_conveyor.data = 10000
        elif self.key_states['x']:
            self.run_conveyor.data = 0
        elif self.key_states['v']:
            self.run_conveyor.data = -10000
        
        if self.run_conveyor.data != self.prev_run_conveyor:
            self.run_conveyor_pub.publish(self.run_conveyor)
            self.prev_run_conveyor = self.run_conveyor.data

        # Conveyor Plungy Stuff
        if self.key_states['a']:
            self.plunge_speed.data = 25
        elif self.key_states['d']:
            self.plunge_speed.data = -50
        else:
            self.plunge_speed.data = 0

        if self.plunge_speed.data != self.prev_plunge_speed:
            self.plunge_pub.publish(self.plunge_speed)
            self.prev_plunge_speed = self.plunge_speed.data

        if self.key_states['o']:
            self.open = not self.open
            
        if self.open != self.prev_open:
            self.deposit_open.data = self.open
            self.dump_pub.publish(self.deposit_open)
            self.prev_open = self.open

if __name__ == '__main__':
    try:
        key_control_node = KeyControlNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass