#this is bob
#his job is to take one publishers stuff
#and put it into a subscribers mailbox
#he profits from this by being the middle man and reselling the publishers work at a markup
#bob sucks
#but the publisher refuses to change his ways
#so bob continues to profit

#Bob now has other uses, I wrote the above lines back in B-term its now C-term and I am using Bob as a more purposful middleman.
#put simply Bob now recives one newspaper from one publisher and two from another and bob interleaves them together to provide one newspaper to the end user
#(I am combining the acceleration and velocity readings into one.)

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from sensor_msgs.msg import Image
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from rclpy.qos import HistoryPolicy

#taken from and modified from the Tutorials->Beginner:Client Libaries->Writing a simple publisher and subscriber (python)
#any else you see here that isnt in there, and I am not kidding here. I read through the libraries code because there wasn't documentation on it
class bob(Node):

    currGyroReading : Imu
    previousCombinedReading : Imu 

    def __init__(self):
        super().__init__('bob_node')
        QoSOverride = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_ALL)
    
        #used to be one now it is two because the IMU topic doesn't output anything and instead it comes from the two topics below.
        self.accel_Subscription = self.create_subscription(
            Imu,
            "/accel/sample",
            self.accel_callback,
            10
        )
        self.gyro_Subscription = self.create_subscription(
            Imu,
            "/gyro/sample",
            self.gyro_callback,
            10
        )
        self.accel_Subscription.qos_profile = QoSOverride
        self.gyro_Subscription.qos_profile = QoSOverride
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
    def gyro_callback(self, msg):
        self.currGyroReading = msg.data
    def accel_callback(self, msg : Imu):
        # If the current acceleration reading and the gyroscope reading time stamps are in the past or at the current time stamp.
        if self.previousCombinedReading.header.stamp < msg.header.stamp and self.currGyroReading.header.stamp <= msg.header.stamp:
            #add the gyroscope readings to the acceleration IMU message
            msg.angular_velocity.x = self.currGyroReading.angular_velocity.x
            msg.angular_velocity.y = self.currGyroReading.angular_velocity.y
            msg.angular_velocity.z = self.currGyroReading.angular_velocity.z
            self.IMU_publisher.publish(msg) #then send the modified message to the publisher to publish to the correct topic via the IMU publisher

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
