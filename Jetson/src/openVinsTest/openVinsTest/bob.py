#this is bob
#his job is to take one publishers stuff
#and put it into a subscribers mailbox
#he profits from this by being the middle man and reselling the publishers work at a markup
#bob sucks
#but the publisher refuses to change his ways
#so bob continues to profit

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from sensor_msgs.msg import Image
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from rclpy.qos import HistoryPolicy

#taken from and modified from the Tutorials->Beginner:Client Libaries->Writing a simple publisher and subscriber (python)
#any else you see here that isnt in there I and I am not kidding here. I read through the libraries code because there wasn't documentation on the stuff
class bob(Node):
    def __init__(self):
        super().__init__('bob_node')
        QoSOverride = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_ALL)
    
        self.IMU_Subscription = self.create_subscription(
            Imu,
            '/imu',
            self.IMU_callback,
            10
        )
        self.IMU_Subscription.qos_profile = QoSOverride
        self.Cam_Subscription = self.create_subscription(
            Image,
            '/infra1/image_rect_raw',
            self.cam_callback,
            10
        )
        self.Cam_Subscription.qos_profile = QoSOverride
        self.IMU_publisher = self.create_publisher(
            Imu,
            "/imu0",
            10
        )
        self.cam_publisher = self.create_publisher(
            Image,
            "/cam0/image_raw",
            10
        )
    #end of init
    def IMU_callback(self, msg):
        self.IMU_publisher.publish(msg)
    def cam_callback(self, msg):
        self.cam_publisher.publish(msg)
def main(args=None):
    rclpy.init(args=args)
    theBob = bob()
    theBob.get_logger().info("============================BOB LIVES============================")
    rclpy.spin(theBob)

    #after a while
    #take him out back
    theBob.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()
