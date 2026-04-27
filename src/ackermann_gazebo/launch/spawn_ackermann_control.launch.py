from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
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

    control_config = PathJoinSubstitution([
        FindPackageShare("ackermann_control"),
        "config",
        "ros2_control.yaml"
    ])

    gazebo_launch_file = PathJoinSubstitution([
        FindPackageShare("gazebo_ros"),
        "launch",
        "gazebo.launch.py"
    ])

    robot_description = ParameterValue(
        Command([
            "xacro ",
            xacro_file,
            " use_ros2_control:=true",
            " control_config:=",
            control_config
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
            "-z", "0.5"
        ]
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager"
        ]
    )

    front_steer_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            "front_steer_position_controller",
            "--controller-manager",
            "/controller_manager"
        ]
    )

    rear_wheel_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            "rear_wheel_velocity_controller",
            "--controller-manager",
            "/controller_manager"
        ]
    )

    spawn_controllers_after_robot = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_entity,
            on_exit=[
                TimerAction(
                    period=3.0,
                    actions=[
                        joint_state_broadcaster_spawner,
                        front_steer_controller_spawner,
                        rear_wheel_controller_spawner
                    ]
                )
            ]
        )
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        TimerAction(period=3.0, actions=[spawn_entity]),
        spawn_controllers_after_robot
    ])
