from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 车辆模型的 xacro 文件。xacro 可以理解为“带变量/宏的 URDF”，
    # launch 时会先被 xacro 命令展开成普通 URDF 字符串。
    xacro_file = PathJoinSubstitution([
        FindPackageShare("ackermann_description"),
        "urdf",
        "ackermann_car.urdf.xacro"
    ])

    # Gazebo 的世界文件，描述仿真环境本身。
    # 当前 empty.world 只加载了地面 ground_plane 和太阳光 sun。
    world_file = PathJoinSubstitution([
        FindPackageShare("ackermann_gazebo"),
        "worlds",
        "empty.world"
    ])

    # ros2_control 的控制器配置文件：
    # 里面声明 controller_manager 要加载哪些控制器，以及这些控制器控制哪些关节。
    control_config = PathJoinSubstitution([
        FindPackageShare("ackermann_control"),
        "config",
        "ros2_control.yaml"
    ])

    # gazebo_ros 是 ROS 官方/社区提供的 Gazebo-ROS 桥接包，不是本工程自己写的。
    # 它自带 gazebo.launch.py，用来启动 Gazebo，并把 Gazebo 接入 ROS 2。
    gazebo_launch_file = PathJoinSubstitution([
        FindPackageShare("gazebo_ros"),
        "launch",
        "gazebo.launch.py"
    ])

    # 运行命令：
    #   xacro ackermann_car.urdf.xacro use_ros2_control:=true control_config:=...
    # 得到完整 URDF 文本，并作为 robot_description 参数传给 robot_state_publisher。
    # use_ros2_control:=true 会让 xacro 额外生成 ros2_control 和 gazebo_ros2_control 插件配置。
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

    # IncludeLaunchDescription 表示“把另一个 launch 文件也启动起来”。
    # 这里实际启动的是 gazebo_ros 包里的 gazebo.launch.py，并把 world_file 传给它。
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_file),
        launch_arguments={
            "world": world_file
        }.items()
    )

    # robot_state_publisher 读取 robot_description 中的 URDF，根据关节状态发布 TF。
    # fixed joint 会形成静态 TF；可动 joint 的 TF 需要 joint_states 持续提供关节位置。
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

    # spawn_entity.py 是 gazebo_ros 提供的工具脚本。
    # 它从 /robot_description topic 读取 URDF/SDF，把名为 ackermann_car 的模型插入 Gazebo。
    # -x/-y/-z 是模型出生时在 Gazebo 世界坐标系下的位置。
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

    # spawner 是 controller_manager 包提供的命令行工具。
    # 它不是控制器本体，而是“请求 /controller_manager 加载并启动某个控制器”。
    # joint_state_broadcaster 会把 ros2_control 读到的各关节状态发布成 /joint_states。
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

    # 前轮转向控制器：对应 ros2_control.yaml 中的
    # front_steer_position_controller，控制两个前轮转向关节的位置。
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

    # 后轮速度控制器：对应 ros2_control.yaml 中的
    # rear_wheel_velocity_controller，控制两个后轮关节的转速。
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

    # 等 spawn_entity 进程退出后，再等 3 秒加载控制器。
    # 原因是 /controller_manager 由 Gazebo 中的 gazebo_ros2_control 插件创建，
    # 这个插件只有在机器人模型被成功插入 Gazebo 后才会存在。
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

    # 启动顺序：
    # 1. 启动 Gazebo
    # 2. 启动 robot_state_publisher，发布 robot_description/TF
    # 3. 延迟 3 秒把车模型 spawn 到 Gazebo
    # 4. 车 spawn 完成后，再延迟 3 秒加载控制器
    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        TimerAction(period=3.0, actions=[spawn_entity]),
        spawn_controllers_after_robot
    ])
