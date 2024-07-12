import rclpy
from rclpy.clock import Clock
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


class MyLidarLocalization(Node):

    def __init__(self):
        super().__init__('my_lidar_localization')
        qos_profile = QoSProfile(depth=10)
        self.lidar_subscriber = self.create_subscription(
            LaserScan, 
            '/scan', 
            self.subscribe_lidar_message, 
            qos_profile)

        self.localization_output_publisher = self.create_publisher(String, '/my_lidar_localization_output', qos_profile)
        self.timer = self.create_timer(1, self.publish_lidar_localization_output_msg)


    
    def subscribe_lidar_message(self, msg):
        self.get_logger().info('Received lidar scan header: {0}'.format(msg.header))


    def publish_lidar_localization_output_msg(self):
        msg = String()
        msg.data = 'Lidar localization output ({0})'.format(Clock().now())
        self.localization_output_publisher.publish(msg)
        self.get_logger().info('Published message: {0}'.format(msg.data))


def main(args=None):
    rclpy.init(args=args)
    node = MyLidarLocalization()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGNINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()