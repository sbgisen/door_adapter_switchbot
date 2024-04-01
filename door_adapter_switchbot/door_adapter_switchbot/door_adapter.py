#!/usr/bin/env pipenv-shebang
# -*- encoding: utf-8 -*-

# Copyright (c) 2024 SoftBank Corp.
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
"""Open-RMF Door Adapter definition for manual doors with Switchbot Contact Sensor attached."""

import argparse
import sys
import threading
import time

import rclpy
import yaml
from DoorClientAPI import DoorClientAPI
from rclpy.node import Node
from rmf_door_msgs.msg import DoorMode
from rmf_door_msgs.msg import DoorRequest
from rmf_door_msgs.msg import DoorState


class DoorAdapter(Node):
    """Open-RMF Door Adapter definition for manual doors with Switchbot Contact Sensor attached."""

    def __init__(self, config_yaml: dict) -> None:
        """Constructor.

        Args:
            config_yaml (dict): The configuration YAML file for this door adapter.

        """
        super().__init__('door_adapter')
        self.get_logger().info('Starting door adapter...')
        self.__door_name = config_yaml['door']['name']
        self.__door_close_feature = config_yaml['door']['door_close_feature']
        self.__door_signal_period = config_yaml['door']['door_signal_period']
        self.__door_state_publish_period = config_yaml['door_publisher']['door_state_publish_period']
        self.__api = DoorClientAPI(config_yaml['door']['device_name'])
        # Default door state - closed mode
        self.__door_mode = DoorMode.MODE_CLOSED
        # open door flag
        self.__open_door = False
        self.__check_status = False  # Door status will be checked by API while this is True.
        self.__door_states_pub = self.create_publisher(
            DoorState, config_yaml['door_publisher']['topic_name'], 10)
        self.create_subscription(
            DoorRequest, config_yaml['door_subscriber']['topic_name'], self.__door_request_cb, 10)
        self.create_timer(self.__door_state_publish_period, self.time_cb)

    def door_open_command_request(self) -> None:
        """Send open command to the door API."""
        while self.__open_door:
            success = self.__api.open_door()
            if success:
                self.get_logger().info(f"Request to open door [{self.__door_name}] is successful")
                # return
            else:
                self.get_logger().warning(f"Request to open door [{self.__door_name}] is unsuccessful")
            time.sleep(self.__door_signal_period)

    def time_cb(self) -> None:
        """Callback function to publish door state at regular intervals.
        Expected to be activated when one once to use the door.

        """
        if self.__check_status:
            self.__door_mode = self.__api.get_mode()
            # when door request is to close door and the door state is close
            # will assume the door state is close until next door open request
            # This implement to reduce the number of API called
            if self.__door_mode == DoorMode.MODE_CLOSED and not self.__open_door:
                self.__check_status = False
        state_msg = DoorState()
        state_msg.door_time = self.get_clock().now().to_msg()
        # publish states of the door
        state_msg.door_name = self.__door_name
        state_msg.current_mode.value = self.__door_mode
        self.__door_states_pub.publish(state_msg)

    def __door_request_cb(self, msg: DoorRequest) -> None:
        """Callback function to handle door requests from the Open-RMF server.
        Check DoorRequest msg whether the door name of the request is same as the current door.
        Ignore the request if not.
        When door node receive open request, the door adapter will send open command to API.
        If door node receive close request, the door adapter will stop sending open command to API.

        Args:
            msg (DoorRequest): The door request message from the Open-RMF server.

        """
        if msg.door_name != self.__door_name:
            return
        self.get_logger().info(f"Door mode [{msg.requested_mode.value}] requested by {msg.requester_id}")
        if msg.requested_mode.value == DoorMode.MODE_OPEN:
            # open door implementation
            self.__open_door = True
            self.__check_status = True
            if self.__door_close_feature:
                self.__api.open_door()
            else:
                t = threading.Thread(target=self.door_open_command_request)
                t.start()
        elif msg.requested_mode.value == DoorMode.MODE_CLOSED:
            # close door implementation
            self.__open_door = False
            self.get_logger().info('Close Command to door received')
            if self.__door_close_feature:
                self.__api.close_door()
        else:
            self.get_logger().error('Invalid door mode requested. Ignoring...')


def main(argv: list = sys.argv) -> None:
    """Run adapter node."""
    rclpy.init(args=argv)
    args_without_ros = rclpy.utilities.remove_ros_args(argv)
    parser = argparse.ArgumentParser(
        prog="door_adapter",
        description="Configure and spin up door adapter for door ")
    parser.add_argument(
        "-c", "--config_file", type=str, required=True,
        help="Path to the config.yaml file for this door adapter")
    args = parser.parse_args(args_without_ros[1:])
    with open(args.config_file, "r") as f:
        config_yaml = yaml.safe_load(f)
    door_adapter = DoorAdapter(config_yaml)
    rclpy.spin(door_adapter)
    door_adapter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main(sys.argv)
