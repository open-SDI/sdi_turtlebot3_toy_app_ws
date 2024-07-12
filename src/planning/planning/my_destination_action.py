# Copyright 2021 OROCA
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
import argparse
import sys

from action_msgs.msg import GoalStatus
from ros_node_interface.action import Destination
from rclpy.action import ActionClient
from rclpy.node import Node


class MyDestinationAction(Node):

    def __init__(self):
        super().__init__('my_destination_action')
        self.destination_action_client = ActionClient(       # Action clieng definition
          self,
          Destination,        # Action type
          'destination')     # Action name

    def send_destination(self, destination):
        wait_count = 1
        while not self.destination_action_client.wait_for_server(timeout_sec=0.1):       # Make connection to action server
            if wait_count > 3:
                self.get_logger().warning('Destination action server is not available.')
                return False
            wait_count += 1
        goal_msg = Destination.Goal()     # Define action goal
        goal_msg.destination = destination
        self.send_goal_future = self.destination_action_client.send_goal_async(      # Send action request (goal)
            goal_msg,       # Action goal
            feedback_callback=self.get_destination_action_feedback)      # Set action FEEDBACK CALLBACK funtion
        self.send_goal_future.add_done_callback(self.get_destination_action_goal)    # Set action STATUS CALLBACK function
        return True

    def get_destination_action_goal(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warning('Action goal rejected.')
            return
        self.get_logger().info('Action goal accepted.')
        self.action_result_future = goal_handle.get_result_async()      # Get action result
        self.action_result_future.add_done_callback(self.get_destination_action_result)      # Set action RESULT CALLBACK function if goal accepted

    def get_destination_action_feedback(self, feedback_msg):     # Action feedback callback
        action_feedback = feedback_msg.feedback.diff_to_destination
        self.get_logger().info('Action feedback: {0}'.format(action_feedback))

    def get_destination_action_result(self, future):     # Action result callback
        action_status = future.result().status
        action_result = future.result().result
        if action_status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Action succeeded!')
            self.get_logger().info(
                'Action result(diff_to_destination): {0}'.format(action_result.diff_to_destination))
        else:
            self.get_logger().warning(
                'Action failed with status: {0}'.format(action_status))
            

def main(argv=sys.argv[1:]):        # Get argument
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)        # Parse arguments
    parser.add_argument(
        '-d',
        '--destination',
        type=str,
        default='401',
        help='Destination address')
    parser.add_argument(
        'argv', nargs=argparse.REMAINDER,
        help='Pass arbitrary arguments to the executable')
    args = parser.parse_args()

    rclpy.init(args=args.argv)
    try:
        checker = MyDestinationAction()
        checker.send_destination(args.destination)    # Send action request (goal)
        checker.get_logger().info('Action goal sent: {0}.'.format(args.destination))
        try:
            rclpy.spin(checker)
        except KeyboardInterrupt:
            checker.get_logger().info('Keyboard Interrupt (SIGINT)')
        finally:
            checker.destination_action_client.destroy()      # Action client destroy
            checker.destroy_node()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()