#    Copyright 2026
#
#    Licensed under the Apache License, Version 2.0 (the "License");
#    you may not use this file except in compliance with the License.
#    You may obtain a copy of the License at
#
#        http://www.apache.org/licenses/LICENSE-2.0

"""Simple launch file for the Cia410Driver against a CiA 410 inclinometer.

Expects bus.yml, master.dcf, and the per-slave .bin file(s) to live in a single
directory (default /data/ros2_canopen). Override with --ros-args or as a launch
argument:

    ros2 launch canopen_410_driver cia410.launch.py \
        config_dir:=/data/ros2_canopen \
        can_interface:=canE
"""

import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    config_dir_arg = DeclareLaunchArgument(
        "config_dir",
        default_value="/data/ros2_canopen",
        description="Directory containing bus.yml and the dcfgen-generated master.dcf.",
    )

    can_interface_arg = DeclareLaunchArgument(
        "can_interface",
        default_value="canE",
        description="CAN interface used by master and driver (e.g. canE, can0, vcan0).",
    )

    master_bin_arg = DeclareLaunchArgument(
        "master_bin",
        default_value=[LaunchConfiguration("config_dir"), "/master.bin"],
        description="Path to master.bin (defaults to <config_dir>/master.bin).",
    )

    namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value="",
        description="Optional namespace for the device container node.",
    )

    config_dir = LaunchConfiguration("config_dir")
    can_interface = LaunchConfiguration("can_interface")
    master_bin = LaunchConfiguration("master_bin")
    namespace = LaunchConfiguration("namespace")

    device_container = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory("canopen_core"), "launch"),
                "/canopen.launch.py",
            ]
        ),
        launch_arguments={
            "bus_config": [config_dir, "/bus.yml"],
            "master_config": [config_dir, "/master.dcf"],
            "master_bin": master_bin,
            "can_interface_name": can_interface,
            "namespace": namespace,
        }.items(),
    )

    return LaunchDescription(
        [
            config_dir_arg,
            can_interface_arg,
            master_bin_arg,
            namespace_arg,
            device_container,
        ]
    )
