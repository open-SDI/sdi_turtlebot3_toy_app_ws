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
from rclpy.node import Node
from ros_node_interface.srv import EmergencyControl


class MyEmergencyService(Node):       # Node definition
    def __init__(self):
        super().__init__('my_emergency_service')        # Node super class generator, node name
        self.emergency_flag = True

        self.emergency_service_client = self.create_client(    # Service client generation
            EmergencyControl,     # Service type
            'emergency_control')  # Service name

        while not self.emergency_service_client.wait_for_service(timeout_sec=0.1):         # Service client request loop
            self.get_logger().warning('The emergency_control service not available.')


    def send_request(self):
        service_request = EmergencyControl.Request()      # Service request instantiation
        service_request.emergency_control_request = self.emergency_flag
        self.emergency_flag = not self.emergency_flag
        futures = self.emergency_service_client.call_async(service_request)        # Request service and get result holder
        return futures      # Return service result holder


def main(args=None):
    rclpy.init(args=args)
    emergencyService = MyEmergencyService()
    future = emergencyService.send_request()    # Set service result holder
    user_trigger = True
    try:
        while rclpy.ok():
            if user_trigger is True:
                rclpy.spin_once(emergencyService)       # Request service
                if future.done():
                    try:
                        service_response = future.result()      # Get service result
                    except Exception as e:  # noqa: B902
                        emergencyService.get_logger().warn('Service call failed: {}'.format(str(e)))
                    else:
                        emergencyService.get_logger().info(
                            'Result: {}'.format(service_response.control_holder))        # Use service result
                        user_trigger = False        # Set flag to wait another user request
            else:
                input('Press Enter for next service call.')     # Wait a new user request
                future = emergencyService.send_request()
                user_trigger = True

    except KeyboardInterrupt:
        emergencyService.get_logger().info('Keyboard Interrupt (SIGINT)')

    emergencyService.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()