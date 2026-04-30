# Ackermann From Scratch Workspace

这是一个从零搭建的 ROS 2 Ackermann 小车仿真工作空间，当前包含车辆 URDF/xacro 建模、RViz 显示、Gazebo 生成、ros2_control 控制器配置，以及 `/cmd_vel` 到前轮转角和后轮速度命令的转换节点。

项目会持续更新。这个 README 用来记录当前工程结构、已经完成的工作、常用启动方式和后续维护入口。

## Package Overview

### ackermann_description

车辆描述包，负责小车模型、TF 结构和 RViz 显示。

主要内容：

- `urdf/ackermann_car.urdf.xacro`：Ackermann 小车模型。
- `launch/display.launch.py`：启动 `robot_state_publisher`、`joint_state_publisher_gui` 和 RViz。
- `rviz/display.rviz`：RViz 显示配置。

当前模型特点：

- 使用 `base_footprint -> base_link` 的 TF 结构。
- `base_footprint` 表示车辆在地面的投影参考点。
- `base_link` 位于车身中心，车身 visual/collision/inertial 都以自身中心为原点。
- 前轮通过 `front_left_steer_joint`、`front_right_steer_joint` 转向。
- 后轮通过 `rear_left_wheel_joint`、`rear_right_wheel_joint` 驱动。
- 雷达 `laser_link` 固定在车身上方。
- 在 `base_link` 中增加了前后灰色可视轴杆，用于让车身和左右轮视觉上连起来。
- 车身底部已抬离地面，避免底盘和轮子同时接触地面导致 Gazebo 转弯阻力异常。

### ackermann_gazebo

Gazebo 仿真包，负责启动世界、发布机器人描述并将小车生成到 Gazebo。

主要内容：

- `worlds/empty.world`：空世界，包含基础地面和光照。
- `launch/spawn_ackermann.launch.py`：只生成小车模型，不加载 ros2_control 控制器。
- `launch/spawn_ackermann_control.launch.py`：生成小车模型，并加载 ros2_control 控制器。

当前设置：

- `spawn_entity.py` 的初始高度为 `-z 0.02`。
- 控制版 launch 会使用 xacro 参数 `use_ros2_control:=true`。
- 控制器在小车生成后延迟加载，避免 `/controller_manager` 尚未创建时加载失败。

### ackermann_control

控制配置和命令转换包，负责 ros2_control 控制器配置，以及把 `/cmd_vel` 转换成具体关节命令。

主要内容：

- `config/ros2_control.yaml`：控制器配置。
- `scripts/cmd_vel_to_ackermann.py`：订阅 `/cmd_vel`，发布前轮转角和后轮速度。

当前控制器：

- `joint_state_broadcaster`：发布 `/joint_states`。
- `front_steer_position_controller`：控制两个前轮转向关节的位置。
- `rear_wheel_velocity_controller`：控制两个后轮关节速度。

命令转换节点行为：

- 输入：`geometry_msgs/Twist`，话题 `/cmd_vel`。
- 输出：
  - `/front_steer_position_controller/commands`
  - `/rear_wheel_velocity_controller/commands`
- `linear.x` 接近 0 时会停车，因为 Ackermann 车不能像差速车一样原地旋转。
- `angular.z` 会根据 Ackermann 几何转换成前轮转角和左右后轮速度差。

## Build

在工作空间根目录执行：

```bash
colcon build
source install/setup.bash
```

如果只是修改 launch、xacro、yaml 或 Python 脚本，重新构建并重新 source 可以确保 install 空间使用最新文件。

## Run

### RViz 查看模型

```bash
ros2 launch ackermann_description display.launch.py
```

### Gazebo 只生成模型

```bash
ros2 launch ackermann_gazebo spawn_ackermann.launch.py
```

### Gazebo + ros2_control

```bash
ros2 launch ackermann_gazebo spawn_ackermann_control.launch.py
```

### 启动 cmd_vel 转 Ackermann 命令节点

```bash
ros2 run ackermann_control cmd_vel_to_ackermann.py
```

### 发布测试速度

Ackermann 车不能原地旋转，测试转弯时需要同时给前进速度和角速度：

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}, angular: {z: 0.4}}"
```

停车：

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}"
```

## Current Progress

目前已经完成：

- 搭建 `ackermann_description`、`ackermann_gazebo`、`ackermann_control` 三个包。
- 建立四轮 Ackermann 小车 URDF/xacro 模型。
- 增加 `base_footprint -> base_link` TF 结构。
- 将 `base_link` 几何体调整为以自身中心为原点。
- 修正车身和轮子高度关系，避免底盘擦地。
- 修正雷达高度，使 `laser_link` 贴在车身上方。
- 增加前后轴杆 visual，提高模型可读性。
- 配置 Gazebo 生成 launch。
- 配置 ros2_control 的转向位置控制器和后轮速度控制器。
- 编写 `/cmd_vel` 到 Ackermann 控制命令的转换节点。
- 将 Gazebo spawn 初始高度调整为 `0.02`。

## Notes

- 当前模型使用简化圆柱轮胎，Gazebo 接触和摩擦参数后续还可以继续优化。
- 当前控制方式是前轮转向、后轮驱动。
- 目前 `/cmd_vel.angular.z` 被解释为期望车身角速度，不是直接的前轮转角。
- 如果给 `linear.x = 0` 且只给 `angular.z`，车辆不会原地转向，这是 Ackermann 车辆模型的正常约束。

## Git Workflow

常用提交流程：

```bash
git status
git add .
git status
git commit -m "Describe the current change"
git push
```

如果不想提交某些本地文件，先用 `git status` 检查，再选择性 add：

```bash
git add README.md src/ackermann_description/urdf/ackermann_car.urdf.xacro
```

注意：`git commit "message"` 不是正确写法，提交信息需要使用 `-m`：

```bash
git commit -m "message"
```

