from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.substitutions import FindPackageShare

import yaml

def launch_setup(context, *args, **kwargs):

    can_interface_name = LaunchConfiguration("can_interface_name")

    nodes_to_start = []

    # Launch slaves nodes

    slave_launch = PathJoinSubstitution(
        [FindPackageShare("canopen_fake_slaves"), "launch", "cia402_slave.launch.py"]
    )

    slave_dcf = PathJoinSubstitution([
        FindPackageShare("panda_canopen"), "config", "cia402",  "cia402_slave.eds"]
    )

    bus_config = PathJoinSubstitution(
        [FindPackageShare("panda_canopen"), "config", "cia402", "bus.yml"]
    )

    with open(bus_config.perform(context), 'r') as f:
        bus_config_dict = yaml.safe_load(f)

    joints = []
    for key in bus_config_dict.keys():
        if "joint" in key:
            joints.append(key)

    for joint in joints:
        nodes_to_start.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(slave_launch),
                launch_arguments={
                    "node_id": str(bus_config_dict[joint]["node_id"]),
                    "node_name": f"slave_node_{joint}",
                    "slave_config": slave_dcf,
                    "can_interface_name": can_interface_name,
                }.items(),
            )
        )

    # Launch master node

    master_launch = PathJoinSubstitution(
        [FindPackageShare("canopen_core"), "launch", "canopen.launch.py"]
    )

    master_config = PathJoinSubstitution(
        [FindPackageShare("panda_canopen"), "config", "cia402", "master.dcf"]
    )

    bus_config = PathJoinSubstitution(
        [FindPackageShare("panda_canopen"), "config", "cia402", "bus.yml"]
    )
    
    nodes_to_start.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(master_launch),
            launch_arguments={
                "master_config": master_config,
                "master_bin": "",
                "bus_config": bus_config,
                "can_interface_name": can_interface_name,
            }.items(),
        )
    )

    return nodes_to_start

def generate_launch_description():
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "can_interface_name",
            default_value="vcan0",
            description="CAN interface name to run the master and, when in simulation, the fake slaves.",
        )
    )
    
    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])