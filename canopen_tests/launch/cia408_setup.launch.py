#    Copyright 2026
#
#    Licensed under the Apache License, Version 2.0 (the "License");
#    you may not use this file except in compliance with the License.
#    You may obtain a copy of the License at
#
#        http://www.apache.org/licenses/LICENSE-2.0
import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # TODO: Once a CiA 408 fake slave is added under canopen_fake_slaves/, include it here.
    # For now, run against a real Danfoss device on the bus, or wire up your own slave.

    master_bin_path = os.path.join(
        get_package_share_directory("canopen_tests"),
        "config",
        "cia408",
        "master.bin",
    )
    if not os.path.exists(master_bin_path):
        master_bin_path = ""

    device_container = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory("canopen_core"), "launch"),
                "/canopen.launch.py",
            ]
        ),
        launch_arguments={
            "master_config": os.path.join(
                get_package_share_directory("canopen_tests"),
                "config",
                "cia408",
                "master.dcf",
            ),
            "master_bin": master_bin_path,
            "bus_config": os.path.join(
                get_package_share_directory("canopen_tests"),
                "config",
                "cia408",
                "bus.yml",
            ),
            "can_interface_name": "vcan0",
        }.items(),
    )

    return LaunchDescription([device_container])
