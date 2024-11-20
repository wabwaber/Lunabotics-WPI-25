import rclpy as ros
from rclpy.node import Node
from sensor_msgs.msg import Imu

class imuSub(Node):
    def __init__(self):
        super().__init__('imu_sub')
        self.subscription = self.create_subscription(Imu, '/camera/imu', self.imu_callback, 10)
        self.subscription

    def imu_callback(self, msg):
        self.get_logger().info('IMU DATA: %f' % (msg.header.stamp * (1 ** (-9)))) 
        self.get_logger().info("Linear Acceleration: %f" % msg.data.linear_acceleration.fget())
        self.get_logger().info("Angular Velocity: %f" % msg.angular_velocity.fget())
        self.get_logger().info("EOM")

def main():
    ros.init()
    imuNode = imuSub()

    ros.spin(imuNode)

    imuSub.destroy_node()
    ros.shutdown()
    # Keep the node running

if __name__ == '__main__':
    main()