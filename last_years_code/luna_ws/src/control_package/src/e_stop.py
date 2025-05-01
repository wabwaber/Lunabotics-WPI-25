import rospy
from std_msgs.msg import Bool
from pynput import keyboard

class KeyControlNode:
    def __init__(self):
        rospy.init_node('e_stop', anonymous=True)

        # Define publishers for different key presses
        self.e_stop_pub = rospy.Publisher('/robot/e_stop', Bool, queue_size=10)

        # Create a listener for keyboard events
        self.listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        self.listener.start()


        self.enable_localizer = Bool()

        # Initialize variables to track key states
        self.key_states = {
            'q': False,
            'e': False,
            'l': False
        }

        # Create a timer to check key presses periodically
        self.timer = rospy.Timer(rospy.Duration(0.1), self.check_key_presses)

        # Localizer Enable
        self.e_stop = False

        self.localizer_enable_pub = rospy.Publisher('/localizer/enable', Bool, queue_size=10)

    def on_press(self, key):
        try:
            key_char = key.char
            if key_char in self.key_states:
                self.key_states[key_char] = True
        except AttributeError:
            pass

    def on_release(self, key):
        try:
            key_char = key.char
            if key_char in self.key_states:
                self.key_states[key_char] = False
        except AttributeError:
            pass

    def check_key_presses(self, event):
        # Keys needed for driving forward and backward
        if self.key_states['q']:
            self.e_stop = True
            self.e_stop_pub.publish(self.e_stop)
            print("feed hold")
        elif self.key_states['e']:
            self.e_stop = False
            self.e_stop_pub.publish(self.e_stop)
            print("cycle start")
        elif self.key_states['l']:
            self.enable_localizer.data = True
            self.localizer_enable_pub.publish(self.enable_localizer)
        

        

if __name__ == '__main__':
    try:
        key_control_node = KeyControlNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass