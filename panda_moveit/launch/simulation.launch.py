from launch import LaunchDescription
from launch.actions import OpaqueFunction, DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from moveit_configs_utils import MoveItConfigsBuilder

from srdfdom.srdf import SRDF

import yaml


def launch_setup(context, *args, **kwargs):

    # Initialize arguments
    can_interface_name = LaunchConfiguration("can_interface_name")

    # List of nodes to be started
    nodes_to_start = []

    # Get panda configs (defined in urdf.xacro, srdf, ros2_control.xacro)
    moveit_config = (
        MoveItConfigsBuilder("panda", package_name="panda_moveit")
        .robot_description(mappings={"can_interface_name": str(can_interface_name.perform(context))})
        .to_moveit_configs()
    )

    # Generate static virtual joint tfs
    for _, xml_contents in moveit_config.robot_description_semantic.items():
        srdf = SRDF.from_xml_string(xml_contents)
        for vj in srdf.virtual_joints:
            nodes_to_start.append(
                Node(
                    package="tf2_ros",
                    executable="static_transform_publisher",
                    name=f"static_transform_publisher_{vj.name}",
                    output="both",
                    arguments=[
                        "--frame-id",
                        vj.parent_frame,
                        "--child-frame-id",
                        vj.child_link,
                    ],
                )
            )

    # Given the published joint states, publish tf for the robot links
    nodes_to_start.append(
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            respawn=True,
            output="both",
            parameters=[
                moveit_config.robot_description
            ],
        )
    )

    # Move Group
    nodes_to_start.append(
        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="both",
            parameters=[
                moveit_config.to_dict(),
                {
                    "publish_robot_description_semantic": True,
                    "allow_trajectory_execution": True,
                    "capabilities": "",
                    "disable_capabilities": "",
                    "publish_planning_scene": True,
                    "publish_geometry_updates": True,
                    "publish_state_updates": True,
                    "publish_transforms_updates": True,
                    "monitor_dynamics": False,
                },
            ],
        )
    )

    # Controller manager
    ros2_controllers_config = PathJoinSubstitution(
        [FindPackageShare("panda_moveit"), "config", "ros2_controllers.yaml"]
    )

    nodes_to_start.append(
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            output="both",
            parameters=[
                moveit_config.robot_description,
                ros2_controllers_config
            ]
        )
    )

    # Generate spawn controllers
    controller_names = moveit_config.trajectory_execution.get(
        "moveit_simple_controller_manager", {}
    ).get("controller_names", [])
    for controller in controller_names + ["joint_state_broadcaster"]:
        nodes_to_start.append(
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[controller],
                output="screen",
            )
        )

    # Rviz
    nodes_to_start.append(
        Node(
            package="rviz2",
            executable="rviz2",
            respawn=False,
            output="both",
            arguments=[
                "-d",
                PathJoinSubstitution(
                    [
                        FindPackageShare("panda_moveit"),
                        "config",
                        "moveit.rviz",
                    ]
                ),
            ],
            parameters=[
                moveit_config.planning_pipelines,
                moveit_config.robot_description_kinematics,
            ],
        )
    )

    # Launch slaves nodes

    slave_launch = PathJoinSubstitution(
        [FindPackageShare("canopen_fake_slaves"), "launch", "cia402_slave.launch.py"]
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
                    "slave_config": PathJoinSubstitution([
                         FindPackageShare("panda_canopen"), "config", "cia402", str(bus_config_dict[joint]["dcf"])
                    ]),
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
