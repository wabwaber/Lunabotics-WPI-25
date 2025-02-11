#this is bob
#his job is to take one publishers stuff
#and put it into a subscribers mailbox
#he profits from this by being the middle man and reselling the publishers work at a markup
#bob sucks
#but the publisher refuses to change his ways
#so bob continues to profit

#Bob now has other uses, I wrote the above lines back in B-term its now C-term and I am using Bob as a more purposful middleman.
#put simply Bob now recives one newspaper from one publisher and one more from another and bob interleaves them together to provide one newspaper to the end user
#(I am combining the acceleration and velocity readings into one.)

import rclpy as rp
from rclpy.node import Node
import rclpy.qos
from sensor_msgs.msg import Imu
from sensor_msgs.msg import Image
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy import qos_overriding_options
from rclpy.qos_overriding_options import QoSOverridingOptions
from rclpy.qos_overriding_options import QoSPolicyKind

currGyroReading : Imu = None
previousCombinedReading : Imu = None

#taken from and modified from the Tutorials->Beginner:Client Libaries->Writing a simple publisher and subscriber (python)
#any else you see here that isnt in there, and I am not kidding here. I read through the libraries code because there wasn't documentation on it
class bob(Node):


    def __init__(self):
        #sleep(30.0) #DEBUG sleep for 30 seconds
        super().__init__('bob_node')
      #DEBUG#  self.get_logger().info(str(QoSOverridingOptions({QoSPolicyKind.RELIABILITY : ReliabilityPolicy.BEST_EFFORT}).policy_kinds))
    
        #used to be one now it is two because the IMU topic doesn't output anything and instead it comes from the two topics below.
        self.accel_Subscription = self.create_subscription(
            Imu,
            "/camera/camera/accel/sample",
            self.accel_callback,
            QoSProfile(history=HistoryPolicy.KEEP_ALL, reliability=ReliabilityPolicy.BEST_EFFORT)
        )
        #self.accel_Subscription.qos_profile.reliability = ReliabilityPolicy.BEST_EFFORT
        #self.get_logger().info(str(self.accel_Subscription.qos_profile.reliability))
        #self.accel_Subscription
        
        self.gyro_Subscription = self.create_subscription(
            Imu,
            "/camera/camera/gyro/sample",
            self.gyro_callback,
            QoSProfile(history=HistoryPolicy.KEEP_ALL, reliability=ReliabilityPolicy.BEST_EFFORT)
        )
        #self.accel_Subscription.qos_profile = QoSOverride
        #self.gyro_Subscription.qos_profile = QoSOverride
        self.Cam_Subscription = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.cam_callback,
            10
        )
        #self.Cam_Subscription.qos_profile = QoSOverride
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
        self.get_logger().info("=========================RECIEVED IMU DATA=========================")
    def cam_callback(self, msg):
        self.cam_publisher.publish(msg)
    def gyro_callback(self, msg):
        self.currGyroReading = msg
        self.get_logger().info("=========================GYRO READING=========================")
    def accel_callback(self, msg : Imu):
        # If the current acceleration reading and the gyroscope reading time stamps are in the past or at the current time stamp.
        self.get_logger().info("=========================Acceleration READING=========================")
        if previousCombinedReading == None:
            return #return early
        if previousCombinedReading.header.stamp < msg.header.stamp:
            #add the gyroscope readings to the acceleration IMU message
            msg.angular_velocity.x = currGyroReading.angular_velocity.x
            msg.angular_velocity.y = currGyroReading.angular_velocity.y
            msg.angular_velocity.z = currGyroReading.angular_velocity.z
            self.IMU_publisher.publish(msg) #then send the modified message to the publisher to publish to the correct topic via the IMU publisher

def main(args=None):
    try:
        rp.init(args=args)
        theBob = bob()
        #QoSOverride = QoSProfile(reliability=ReliabilityPolicy(2), history=HistoryPolicy(2))
        #theBob.accel_Subscription.qos_profile = QoSOverride
        #theBob.get_logger().info(str(theBob.accel_Subscription.qos_profile))
        #theBob.gyro_Subscription.qos_profile = QoSOverride
        theBob.get_logger().info(str(theBob.accel_Subscription.qos_profile.reliability))
        
        #theBob.get_logger().info("==QOS Realibility Policy : " + str(theBob.get_subscriptions_info_by_topic.__getattribute__()))
        theBob.get_logger().info("============================BOB LIVES============================")
        rp.spin(theBob)
        #after a while
        #take him out back
    except(KeyboardInterrupt,ExternalShutdownException): #if a keyboard Interrupt or if another node tells bob to stop
        pass
    theBob.destroy_node() #bob stops
    #rp.shutdown()
if __name__ == '__main__':
    main()
