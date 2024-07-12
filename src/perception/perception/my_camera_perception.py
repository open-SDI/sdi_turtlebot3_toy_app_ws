import rclpy
from rclpy.clock import Clock
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Image
from std_msgs.msg import String
from rclpy.callback_groups import ReentrantCallbackGroup
from ros_node_interface.srv import ObjectDetection
import random
import time


class MyCameraPerception(Node):

    def __init__(self):
        super().__init__('my_camera_perception')
        qos_profile = QoSProfile(depth=10)
        self.callback_group = ReentrantCallbackGroup()      # Callback group definition for parallization
        
        self.camera_subscriber = self.create_subscription(
            Image, 
            '/camera/image_raw',
            self.subscribe_camera_message, 
            qos_profile)
        
        self.object_detection_service_server = self.create_service(       # Service server definition
            ObjectDetection,                       # Service type
            'object_detection',                    # Service name
            self.get_object_detection_request,     # Service server callback function
            callback_group=self.callback_group)     # Callback function parallization group
        
        self.peception_output_publisher = self.create_publisher(String, '/my_camera_perception_output', qos_profile)
        self.timer = self.create_timer(1, self.publish_camera_perception_output_msg)

    
    def subscribe_camera_message(self, msg):
        self.get_logger().info('Received image header {0} ({1},{2})'.format(msg.header, msg.width, msg.height))

    def get_object_detection_request(self, request, response):
        duration = request.duration    # Interprete service request
        objects = []
        for i in range(duration):
            objects.append("car{0}".format(random.randint(0, 10)))
            time.sleep(1)
        self.get_logger().info('Object detection service done!')
        
        response.objects = objects
        return response     # return service respond

    def publish_camera_perception_output_msg(self):
        msg = String()
        msg.data = 'Camera perception output ({0})'.format(Clock().now())
        self.peception_output_publisher.publish(msg)
        self.get_logger().info('Published message: {0}'.format(msg.data))


def main(args=None):
    rclpy.init(args=args)
    node = MyCameraPerception()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGNINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()