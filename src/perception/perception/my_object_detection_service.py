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

import random

import rclpy
import argparse
import sys

from rclpy.node import Node
from ros_node_interface.srv import ObjectDetection


class MyObjectDetectionService(Node):       # Node definition
    def __init__(self):
        super().__init__('my_object_detection_service')        # Node super class generator, node name
        self.emergency_flag = True

        self.object_detection_service_client = self.create_client(    # Service client generation
            ObjectDetection,     # Service type
            'object_detection')  # Service name

        while not self.object_detection_service_client.wait_for_service(timeout_sec=0.1):         # Service client request loop
            self.get_logger().warning('The object_detection service not available.')


    def send_request(self, duration):
        service_request = ObjectDetection.Request()      # Service request instantiation
        service_request.duration = duration       # duration input TODO
        futures = self.object_detection_service_client.call_async(service_request)        # Request service and get result holder
        return futures      # Return service result holder


def main(argv=sys.argv[1:]):
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)        # Parse arguments
    parser.add_argument(
        '-d',
        '--duration',
        type=int,
        default=10,
        help='Object detection duration')
    parser.add_argument(
        'argv', nargs=argparse.REMAINDER,
        help='Pass arbitrary arguments to the executable')
    args = parser.parse_args()

    rclpy.init(args=args.argv)

    objectDetectionService = MyObjectDetectionService()
    future = objectDetectionService.send_request(args.duration)    # Set service result holder
    user_trigger = True
    try:
        while rclpy.ok():
            if user_trigger is True:
                rclpy.spin_once(objectDetectionService)       # Request service
                if future.done():
                    try:
                        service_response = future.result()      # Get service result
                    except Exception as e:  # noqa: B902
                        objectDetectionService.get_logger().warn('Service call failed: {}'.format(str(e)))
                    else:
                        objectDetectionService.get_logger().info(
                            'Result: {}'.format(service_response.objects))        # Use service result
                        user_trigger = False        # Set flag to wait another user request
            else:
                input('Press Enter for next service call.')     # Wait a new user request
                future = objectDetectionService.send_request(args.duration)
                user_trigger = True

    except KeyboardInterrupt:
        objectDetectionService.get_logger().info('Keyboard Interrupt (SIGINT)')

    objectDetectionService.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()