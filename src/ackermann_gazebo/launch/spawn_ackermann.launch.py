from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    gazebo_ros_pkg = get_package_share_directory("gazebo_ros")

    world_file = PathJoinSubstitution([
        FindPackageShare("ackermann_gazebo"),
        "worlds",
        "empty.world"
    ])

    xacro_file = PathJoinSubstitution([
        FindPackageShare("ackermann_description"),
        "urdf",
        "ackermann_car.urdf.xacro"
    ])

    robot_description = ParameterValue(
        Command(["xacro ", xacro_file]),
        value_type=str
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_pkg, "launch", "gazebo.launch.py")
        ),
        launch_arguments={
            "world": world_file,
            "verbose": "true"
        }.items()
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[
            {"robot_description": robot_description}
        ],
        output="screen"
    )

    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic", "robot_description",
            "-entity", "ackermann_car",
            "-x", "0.0",
            "-y", "0.0",
            "-z", "0.15"
        ],
        output="screen"
    )

    delayed_spawn = TimerAction(
        period=3.0,
        actions=[spawn_entity]
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        delayed_spawn
    ])