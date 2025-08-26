import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_imu_subscriber')
        self.subscription = self.create_subscription(
            Imu,
            'imu',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        # self.get_logger().info('I heard IMU data: linear_acceleration=(%.2f, %.2f, %.2f), angular_velocity=(%.2f, %.2f, %.2f)'
        #                        % (msg.linear_acceleration.x
        #                           , msg.linear_acceleration.y
        #                           , msg.linear_acceleration.z
        #                           , msg.angular_velocity.x
        #                           , msg.angular_velocity.y
        #                           , msg.angular_velocity.z))
        self.get_logger().info('q: (%.2f, %.2f, %.2f, %.2f)'
                               % (msg.orientation.x
                                  , msg.orientation.y
                                  , msg.orientation.z
                                  , msg.orientation.w))


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
