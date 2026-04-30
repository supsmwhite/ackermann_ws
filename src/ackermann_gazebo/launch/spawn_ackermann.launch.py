from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    xacro_file = PathJoinSubstitution([
        FindPackageShare("ackermann_description"),
        "urdf",
        "ackermann_car.urdf.xacro"
    ])

    world_file = PathJoinSubstitution([
        FindPackageShare("ackermann_gazebo"),
        "worlds",
        "empty.world"
    ])

    gazebo_launch_file = PathJoinSubstitution([
        FindPackageShare("gazebo_ros"),
        "launch",
        "gazebo.launch.py"
    ])

    robot_description = ParameterValue(
        Command([
            "xacro ",
            xacro_file
        ]),
        value_type=str
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_file),
        launch_arguments={
            "world": world_file
        }.items()
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "robot_description": robot_description
            }
        ]
    )

    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        output="screen",
        arguments=[
            "-entity", "ackermann_car",
            "-topic", "robot_description",
            "-x", "0",
            "-y", "0",
            "-z", "0.02"
        ]
    )

    delayed_spawn_entity = TimerAction(
        period=3.0,
        actions=[
            spawn_entity
        ]
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        delayed_spawn_entity
    ])
