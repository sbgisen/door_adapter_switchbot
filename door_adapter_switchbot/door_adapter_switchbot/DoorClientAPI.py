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
"""Open-RMF template-based Door Client API used by the corresponding Door Adapter."""

import os
from typing import Optional

from rmf_door_msgs.msg import DoorMode
from switchbot import SwitchBot

SWITCHBOT_TOKEN_ENV = 'SWITCHBOT_TOKEN'
SWITCHBOT_SECRET_ENV = 'SWITCHBOT_SECRET'


class DoorClientAPI:
    """Client for manual doors whose status can be caught by a Switchbot Contact Sensor."""

    def __init__(self, device_name: str) -> None:
        """Constructor.

        Assuming that only one Switchbot account is used among the system, the token and secret
        are set as environment variables. The Switchbot Contact Sensor device should be configured
        using the Switchbot Smartphone App.

        Args:
            device_name (str): The name of the Switchbot Contact Sensor device.

        """
        self.__client = None
        self.__create_client()
        self.__device_id = self.__get_device_id(device_name)
        if self.__device_id is None:
            raise ValueError(f"Device with name {device_name} not found or device is not a Contact Sensor type.")

    def __create_client(self) -> None:
        """Set up the Switchbot client.

        401 status will be caught every 5 minutes after the client was last created.

        """
        token = os.getenv(SWITCHBOT_TOKEN_ENV)
        secret = os.getenv(SWITCHBOT_SECRET_ENV)
        if token is None or secret is None:
            raise ValueError("Please set SWITCHBOT_TOKEN and SWITCHBOT_SECRET environment variables.")
        self.__client = SwitchBot(token=token, secret=secret)

    def __get_device_id(self) -> Optional[str]:
        """Find device ID from the device name. Called once during initialization.

        Returns:
            Optional[str]: The device ID if found and the device is a Contact Sensor type. None otherwise.

        """
        limit_count = 0
        while limit_count < 3:
            try:
                devices = self.__client.devices()
                device = next((device for device in devices if device.name == self.__device_name), None)
                if device is None or device.type != 'Contact Sensor':
                    return None
                return device.id
            except RuntimeError:
                self.__create_client()
                limit_count += 1
        return None

    def __get_door_status(self) -> Optional[str]:
        """Get the door status from the Switchbot API.

        Returns:
            Optional[str]: The string representing door status if the API call is successful. None otherwise.

        """
        limit_count = 0
        while limit_count < 3:
            try:
                status = self.__client.device(self.__device_id).status()
                return status.get('open_state')
            except RuntimeError:
                self.__create_client()
                limit_count += 1

    def open_door(self) -> bool:
        """Command to open the door.

        Returns:
            bool: True after the command is sent.

        """
        # TODO: Tell human to open the door until it's dealt with.
        return True

    def close_door(self) -> bool:
        """Command to close the door.

        Returns:
            bool: True after the command is sent.

        """
        # TODO: Tell human to close the door.
        return True

    def get_mode(self) -> DoorMode:
        """Return the door status with reference rmf_door_msgs.

        Returns: DoorMode.MODE_CLOSED when door status is closed.
            DoorMode.MODE_OPEN when door status is open.
            DoorMode.MODE_UNKNOWN when door status is unknown.

        """
        door_status = self.__get_door_status()
        match door_status:
            case 'open':
                return DoorMode.MODE_OPEN
            case 'close':
                return DoorMode.MODE_CLOSED
            case _:
                return DoorMode.MODE_UNKNOWN
