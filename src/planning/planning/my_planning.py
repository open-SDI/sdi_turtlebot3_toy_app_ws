import rclpy
from rclpy.clock import Clock
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from ros_node_interface.action import Destination
import random
import time


class MyPlanning(Node):

    def __init__(self):
        super().__init__('my_planning')
        qos_profile = QoSProfile(depth=10)
        self.callback_group = ReentrantCallbackGroup()      # Callback group definition for parallization

        self.localization_subscriber = self.create_subscription(
            String, 
            '/my_lidar_localization_output', 
            self.subscribe_localization_message, 
            qos_profile)
        
        self.camera_perception_subscriber = self.create_subscription(
            String, 
            '/my_camera_perception_output', 
            self.subscribe_camera_perception_message, 
            qos_profile)
        
        self.destination_action_server = ActionServer(       # Action server definition
            self,
            Destination,      # Action type
            'destination',   # Action name
            self.destination_checker,   # Action callback function
            callback_group=self.callback_group)     # Callback function parallization group
        
        self.lidar_perception_subscriber = self.create_subscription(
            String, 
            '/my_lidar_perception_output', 
            self.subscribe_lidar_perception_message, 
            qos_profile)

        self.trajectory_publisher = self.create_publisher(String, '/my_trajectory', qos_profile)
        self.timer = self.create_timer(1, self.publish_trajectory_msg)


    def subscribe_localization_message(self, msg):
        self.get_logger().info('Received lidar localization output header {0}'.format(msg.data))

    def subscribe_camera_perception_message(self, msg):
        self.get_logger().info('Received camera perception output header {0}'.format(msg.data))

    def subscribe_lidar_perception_message(self, msg):
        self.get_logger().info('Received lidar perception output header {0}'.format(msg.data))

    def destination_checker(self, goal_handle):
        self.get_logger().info('Execute destination_checker action!')
        feedback_msg = Destination.Feedback()     # Define action feedback holder
        
        cur_location = 0.0
        destination = goal_handle.request.destination     # Action goal interpretation
        distance_to_destination = random.randint(5, 10)

        diff_to_destination = distance_to_destination - cur_location
        while diff_to_destination > 1.0:     # Action process until goal achievement
            feedback_msg.diff_to_destination = diff_to_destination
            self.get_logger().info('Feedback: {0}'.format(feedback_msg.diff_to_destination))
            goal_handle.publish_feedback(feedback_msg)      # Action feedback publish
            time.sleep(1)
            cur_location = cur_location + 1.0
            diff_to_destination = distance_to_destination - cur_location

        goal_handle.succeed()       # Inform goal achievement status
        result = Destination.Result()     # Action result generation
        result.diff_to_destination = diff_to_destination
        
        return result       # Return action result

    def publish_trajectory_msg(self):
        msg = String()
        msg.data = 'Planned trajectory ({0})'.format(Clock().now())
        self.trajectory_publisher.publish(msg)
        self.get_logger().info('Published message: {0}'.format(msg.data))


def main(args=None):
    rclpy.init(args=args)
    node = MyPlanning()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGNINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()