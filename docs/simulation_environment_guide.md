# 仿真环境组成与搭建说明

## 1. 文档目的

本文档用于说明当前工作空间中的双车 Gazebo 数字孪生仿真环境是如何搭建的、由哪些模块组成、各模块之间如何通信，以及启动后数据如何流动。

该仿真环境并不是单纯的 Gazebo 模型展示，而是一个连接实物车、ROS 2 通信、Nav2 定位、Gazebo 模型控制和 RViz 可视化的多模块系统。

整体目标是：

```text
两台实物车的数据进入电脑
电脑在一个 Gazebo 世界中显示两台仿真车
两台仿真车的位置跟随实物车定位
两个 RViz 分别显示两台车的感知和定位状态
```

## 2. 工作空间结构

当前工作空间主要包含三个自定义包：

```text
src/
  mycar_base_sim/
  mycar_sim_bringup/
  wheeltec_robot_urdf/
```

### 2.1 wheeltec_robot_urdf

该包主要负责机器人模型、Gazebo 世界、RViz 配置和 Gazebo 启动。

主要内容：

```text
wheeltec_robot_urdf/
  urdf/
    V550_akm_robot.urdf
  meshes/
    ...
  world/
    my_map.world
  map/
    my_map.yaml
    my_map.pgm
  config/
    ros2_controllers.yaml
  launch/
    display_gazebo.launch.py
  rviz/
    car1.rviz
    car2.rviz
    default.rviz
```

功能包括：

- 读取 URDF。
- 在 Gazebo 中生成机器人模型。
- 生成第二台车使用的带 `robot2_` 前缀的 URDF。
- 启动 Gazebo world。
- 启动 robot_state_publisher。
- 启动 ros2_control 控制器。
- 提供 RViz 配置文件。

### 2.2 mycar_base_sim

该包主要负责数字孪生绑定和跨 domain 话题桥接。

主要内容：

```text
mycar_base_sim/
  src/
    sim_car_tf_twin.cpp
  scripts/
    domain_topic_bridge.py
  config/
    sim_car_tf_twin.yaml
    sim_car_tf_twin_robot2.yaml
  app/
    base_sim_twin.launch.py
    base_sim_twin_robot2.launch.py
    car2_domain_bridge.launch.py
```

功能包括：

- 读取真实车 TF 或 odom。
- 调用 Gazebo `/set_entity_state` 服务移动仿真模型。
- 根据 `/cmd_vel` 发布轮子和转向关节命令。
- 将 robot2 的 domain2 数据桥接到 domain1 的 `/car2/...` 话题。

### 2.3 mycar_sim_bringup

该包负责统一启动仿真系统。

主要内容：

```text
mycar_sim_bringup/
  launch/
    sim_bringup.launch.py
    sim_bringup_no_rviz.launch.py
    sim_bringup_two_cars.launch.py
    sim_bringup_two_cars_no_rviz.launch.py
```

功能包括：

- 启动 Gazebo。
- 启动 robot1 数字孪生绑定节点。
- 启动 robot2 数字孪生绑定节点。
- 启动 car1 RViz。
- 启动 car2 RViz。

## 3. 总体系统架构

系统由三部分组成：

```text
实物车层
通信桥接层
仿真与可视化层
```

### 3.1 实物车层

两台车各自运行自己的底盘、雷达、定位和 Nav2。

robot1：

```text
ROS_DOMAIN_ID=1
/scan
/odom
/tf
/tf_static
/cmd_vel
/robot_description
```

robot2：

```text
ROS_DOMAIN_ID=2
/scan
/odom
/tf
/tf_static
/cmd_vel
/robot_description
```

### 3.2 通信桥接层

robot2 不直接进入 domain1，而是通过桥接节点转发。

桥接节点：

```text
mycar_base_sim/scripts/domain_topic_bridge.py
```

它将 domain2 中 robot2 的原始话题复制到 domain1 中，并加上 `/car2` 前缀。

### 3.3 仿真与可视化层

Gazebo 和两个 RViz 均运行在 domain1。

Gazebo 中包含：

```text
V550_akm_robot
V550_akm_robot_2
```

RViz 包含：

```text
rviz2_car1
rviz2_car2
```

## 4. ROS Domain 设计

本系统采用 domain 隔离来解决两台实物车同名话题冲突问题。

```text
robot1: domain1
robot2: domain2
Gazebo: domain1
RViz: domain1
bridge: 同时连接 domain1 和 domain2
```

如果两台车都在同一个 domain 中发布 `/scan`、`/tf`、`/odom`，电脑端无法区分数据来源。因此 robot2 被放在 domain2，通过桥接节点转成 `/car2/...`。

## 5. Gazebo 世界

Gazebo world 文件位于：

```text
src/wheeltec_robot_urdf/world/my_map.world
```

该 world 描述了仿真环境，包括：

- 地面。
- 墙体。
- 光照。
- 物理参数。

启动时并不是直接使用原始 world 文件，而是在 launch 中读取该 world，并生成一个临时 world：

```text
/tmp/wheeltec_robot_urdf_my_map_with_state.world
```

生成临时 world 的原因是需要自动注入 Gazebo 状态插件：

```xml
<plugin name="gazebo_ros_state" filename="libgazebo_ros_state.so">
  <update_rate>30.0</update_rate>
</plugin>
```

该插件提供：

```text
/get_entity_state
/set_entity_state
```

数字孪生绑定节点通过 `/set_entity_state` 控制 Gazebo 模型位姿。

如果没有该插件，Gazebo 模型不会跟随实物车运动。

## 6. 机器人模型

机器人 URDF 文件：

```text
src/wheeltec_robot_urdf/urdf/V550_akm_robot.urdf
```

该 URDF 描述了：

- 车体 `base_link`
- 轮子 link
- 转向 link
- 雷达 link
- 相机 link
- 控制器 link
- ros2_control 配置

### 6.1 robot1 模型

robot1 使用原始 URDF：

```text
base_link
laser_link
lb_wheel_link
rb_wheel_link
...
```

对应话题：

```text
/robot_description
```

### 6.2 robot2 Gazebo 模型

Gazebo 中第二台车不能直接使用相同 link 名，否则会和第一台车冲突。

因此 launch 中会自动生成一份带 `robot2_` 前缀的 URDF：

```text
robot2_base_link
robot2_laser_link
robot2_lb_wheel_link
...
```

对应话题：

```text
/robot2/robot_description
```

Gazebo 中第二台模型：

```text
V550_akm_robot_2
```

### 6.3 car2 RViz 模型

car2 RViz 不能使用 `/robot2/robot_description`。

原因是 `/robot2/robot_description` 中的 link 名带 `robot2_` 前缀，而 car2 的真实 TF 仍然是：

```text
base_link
laser_link
base_footprint
```

因此 car2 RViz 使用：

```text
/car2/robot_description
```

该描述保持未加前缀的 link 名，用于匹配 `/car2/tf` 中的真实车 TF。

## 7. Gazebo 中两台车的生成

在 `display_gazebo.launch.py` 中使用 `spawn_entity.py` 生成两台车。

robot1：

```text
entity: V550_akm_robot
topic: robot_description
```

robot2：

```text
entity: V550_akm_robot_2
topic: /robot2/robot_description
```

两台车在 Gazebo 中是不同实体。

## 8. Gazebo 控制器

控制器配置文件：

```text
src/wheeltec_robot_urdf/config/ros2_controllers.yaml
```

当前主要使用：

```text
joint_state_broadcaster
rear_wheel_velocity_controller
steering_position_controller
```

数字孪生节点会根据真实车速度指令发布：

```text
/rear_wheel_velocity_controller/commands
/steering_position_controller/commands
```

对于 robot2，则使用：

```text
/robot2/rear_wheel_velocity_controller/commands
/robot2/steering_position_controller/commands
```

需要注意，当前第二台车的轮子动画和控制器支持程度取决于 Gazebo 中第二台模型是否带有独立 ros2_control。当前主要关注的是模型位姿同步。

## 9. 数字孪生绑定节点

核心节点：

```text
sim_car_tf_twin_node
```

源码：

```text
src/mycar_base_sim/src/sim_car_tf_twin.cpp
```

该节点功能：

1. 读取真实车位姿。
2. 对位姿做平滑滤波。
3. 调用 Gazebo `/set_entity_state`。
4. 发布轮子转动和前轮转向命令。

### 9.1 robot1 绑定

配置文件：

```text
src/mycar_base_sim/config/sim_car_tf_twin.yaml
```

关键参数：

```yaml
pose_source: tf
global_frame: map
base_frame: base_footprint
model_name: V550_akm_robot
cmd_vel_topic: /cmd_vel
```

robot1 读取：

```text
/tf 中 map -> base_footprint
```

并控制：

```text
Gazebo V550_akm_robot
```

### 9.2 robot2 绑定

配置文件：

```text
src/mycar_base_sim/config/sim_car_tf_twin_robot2.yaml
```

关键参数：

```yaml
pose_source: tf
global_frame: map
base_frame: base_footprint
model_name: V550_akm_robot_2
cmd_vel_topic: /car2/cmd_vel
```

启动文件中将 TF remap 为：

```text
/tf        -> /car2/tf
/tf_static -> /car2/tf_static
```

因此 robot2 绑定节点读取：

```text
/car2/tf 中 map -> base_footprint
```

并控制：

```text
Gazebo V550_akm_robot_2
```

## 10. 为什么使用 TF 而不是 odom 驱动 Gazebo

早期 robot2 曾使用：

```text
/car2/odom
```

驱动 Gazebo。

这样真实车推动时 Gazebo 会动，但在 RViz 中设置初始位姿时 Gazebo 不会跳转。

原因是 RViz 的 `Set Initial Pose` 会改变 AMCL 的全局定位，也就是影响：

```text
map -> odom_combined
```

但不会改变：

```text
odom_combined -> base_footprint
```

如果 Gazebo 只跟随 odom，就无法反映 AMCL 重定位。

因此最终两台车都使用：

```text
map -> base_footprint
```

驱动 Gazebo。

这样：

- 推动车，Gazebo 会动。
- AMCL 修正定位，Gazebo 会动。
- RViz 设置初始位姿，Gazebo 也会同步跳转。

## 11. RViz 配置

RViz 配置位于：

```text
src/wheeltec_robot_urdf/rviz/
```

### 11.1 car1.rviz

用于显示 robot1。

主要话题：

```text
RobotModel: /robot_description
TF: /tf
LaserScan: /scan
Map: /map
```

### 11.2 car2.rviz

用于显示 robot2。

主要话题：

```text
RobotModel: /car2/robot_description
TF: /car2/tf
LaserScan: /car2/scan
Map: /car2/map
AMCL Pose: /car2/amcl_pose
Particle Cloud: /car2/particle_cloud
Initial Pose: /car2/initialpose
Goal Pose: /car2/goal_pose
```

car2 RViz 中的 initial pose 和 goal pose 不直接发到全局话题，而是发到 `/car2/...`，再由 bridge 转回 domain2。

## 12. 启动文件说明

### 12.1 car2_domain_bridge.launch.py

启动 robot2 domain 桥接：

```bash
ros2 launch mycar_base_sim car2_domain_bridge.launch.py
```

### 12.2 sim_bringup_two_cars.launch.py

启动完整双车仿真：

```bash
ros2 launch mycar_sim_bringup sim_bringup_two_cars.launch.py
```

包含：

- Gazebo。
- robot1 数字孪生绑定。
- robot2 数字孪生绑定。
- car1 RViz。
- car2 RViz。

### 12.3 sim_bringup_two_cars_no_rviz.launch.py

只启动 Gazebo 和绑定节点，不启动 RViz：

```bash
ros2 launch mycar_sim_bringup sim_bringup_two_cars_no_rviz.launch.py
```

适合已有 RViz 或只想测试 Gazebo 时使用。

## 13. 推荐启动流程

### 13.1 启动 robot1

robot1 保持原始启动方式：

```bash
export ROS_DOMAIN_ID=1
```

然后启动 robot1 的底盘、雷达和 Nav2。

### 13.2 启动 robot2

robot2 保持原始启动方式，但使用：

```bash
export ROS_DOMAIN_ID=2
```

然后启动 robot2 的底盘、雷达和 Nav2。

### 13.3 启动桥接节点

电脑端：

```bash
cd /home/j/桌面/work
source install/setup.bash
ros2 launch mycar_base_sim car2_domain_bridge.launch.py
```

### 13.4 启动 Gazebo 和 RViz

电脑端：

```bash
cd /home/j/桌面/work
source install/setup.bash
export ROS_DOMAIN_ID=1
ros2 launch mycar_sim_bringup sim_bringup_two_cars.launch.py
```

## 14. 常用验证命令

### 14.1 检查 car2 桥接话题

```bash
export ROS_DOMAIN_ID=1
ros2 topic list | grep car2
```

应看到：

```text
/car2/scan
/car2/odom
/car2/tf
/car2/tf_static
/car2/map
/car2/amcl_pose
/car2/goal_pose
```

### 14.2 检查 car2 雷达

```bash
ros2 topic echo --once /car2/scan
```

### 14.3 检查 car2 TF

```bash
ros2 run tf2_ros tf2_echo map base_footprint \
  --ros-args -r /tf:=/car2/tf -r /tf_static:=/car2/tf_static
```

### 14.4 检查 Gazebo 状态服务

```bash
ros2 service list | grep entity
```

应看到：

```text
/get_entity_state
/set_entity_state
```

### 14.5 检查 Gazebo 模型列表

```bash
ros2 service call /get_model_list gazebo_msgs/srv/GetModelList "{}"
```

应包含：

```text
V550_akm_robot
V550_akm_robot_2
```

### 14.6 检查 robot2 绑定节点

```bash
ros2 param get /sim_car_tf_twin_robot2 pose_source
ros2 param get /sim_car_tf_twin_robot2 model_name
```

期望：

```text
pose_source: tf
model_name: V550_akm_robot_2
```

## 15. 环境包含的关键能力

当前仿真环境包含：

- 双实物车接入。
- 双 ROS Domain 隔离。
- domain2 到 domain1 的话题桥接。
- 一个 Gazebo 世界显示两台车。
- 两个 RViz 分别显示两台车。
- Gazebo 模型跟随实物车定位。
- car2 RViz 的目标点和初始位姿隔离。
- Gazebo 中两台车实体名隔离。
- RViz 中 robot_description 与 TF frame 对齐。

## 16. 当前环境的局限

当前环境仍有一些限制：

1. robot2 的定位质量取决于车端 AMCL、odom、IMU。
2. 如果 robot2 的 IMU/odom 漂移，Gazebo 也会跟随抖动。
3. 第二台 Gazebo 车主要用于位姿显示，轮子动画和独立 ros2_control 支持仍可继续完善。
4. 地图文件需要同时保证 Gazebo world 和 Nav2 map yaml 对应。
5. 多车数量继续增加时，需要扩展 `/car3/...`、`/car4/...` 等命名空间和 bridge 规则。

## 17. 总结

该仿真环境的核心思想是：

```text
车端保持原样
电脑端通过 domain 和 bridge 做多车隔离
Gazebo 统一显示所有车辆
RViz 分别显示和控制对应车辆
```

其中最关键的设计点是：

```text
robot1 使用原始话题
robot2 通过 /car2 前缀进入 domain1
Gazebo 使用 /set_entity_state 进行数字孪生位姿同步
RViz 模型描述必须与对应 TF frame 名一致
```

这套环境为后续多车编队、协同导航、数字孪生监控和多机器人实验提供了基础框架。
