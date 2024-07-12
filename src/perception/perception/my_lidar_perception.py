import rclpy
from rclpy.clock import Clock
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


class MyLidarPerception(Node):

    def __init__(self):
        super().__init__('my_lidar_perception')
        qos_profile = QoSProfile(depth=10)
        
        self.lidar_subscriber = self.create_subscription(
            LaserScan, 
            '/scan',  
            self.subscribe_lidar_message, 
            qos_profile)
        
        self.lidar_localization_subscriber = self.create_subscription(
            String, 
            '/my_lidar_localization_output',  
            self.subscribe_lidar_localization_message, 
            qos_profile)
        
        self.peception_output_publisher = self.create_publisher(String, '/my_lidar_perception_output', qos_profile)
        self.timer = self.create_timer(1, self.publish_lidar_perception_output_msg)

    
    def subscribe_lidar_message(self, msg):
        self.get_logger().info('Received lidar scan header: {0}'.format(msg.header))

    def subscribe_lidar_localization_message(self, msg):
        self.get_logger().info('Received lidar localization msg: {0}'.format(msg.data))


    def publish_lidar_perception_output_msg(self):
        msg = String()
        msg.data = 'Lidar perception output ({0})'.format(Clock().now())
        self.peception_output_publisher.publish(msg)
        self.get_logger().info('Published message: {0}'.format(msg.data))


def main(args=None):
    rclpy.init(args=args)
    node = MyLidarPerception()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGNINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()