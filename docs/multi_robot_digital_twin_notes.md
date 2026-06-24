# 双实物车与 Gazebo 数字孪生多车系统实验记录

## 1. 项目背景

本项目的目标是在同一台电脑上集成两台实物车、Gazebo 仿真环境和 RViz2 可视化界面，实现一个面向多车编队/多车数字孪生的实验系统。

系统中存在两台实物车：

- `robot1`：第一台实物车，原始系统已经可以稳定运行。
- `robot2`：第二台实物车，后续需要接入同一个电脑端仿真和可视化环境。

电脑端需要运行：

- 一个 Gazebo 世界，显示两台仿真车。
- 两个 RViz2 窗口，分别显示两台车的信息。
- 一个桥接节点，将第二台车的数据从独立 ROS Domain 转发到电脑端统一系统。

最终希望达到的效果是：

```text
robot1 实物车 -> Gazebo 中 V550_akm_robot
robot2 实物车 -> Gazebo 中 V550_akm_robot_2

robot1 RViz 只显示 robot1
robot2 RViz 只显示 robot2

Gazebo 在同一个地图/世界中显示两台仿真车
```

## 2. 初始问题

最开始两台车都在同一个 ROS 2 网络中工作。两台车都会发布类似的话题，例如：

```text
/scan
/odom
/tf
/tf_static
/cmd_vel
/robot_description
```

这带来一个明显问题：如果两台车在同一个 `ROS_DOMAIN_ID` 中同时发布同名话题，电脑端无法区分某条 `/scan` 或 `/tf` 是来自哪一台车。

例如，电脑端订阅 `/scan` 时，可能同时收到两台车的雷达数据。这样不适合做多车系统。

因此，单纯依靠在电脑端“复制话题”并不能解决问题。如果两台车的数据在进入电脑端之前就已经混在一起，那么后续无法可靠地区分来源。

## 3. 最终采用的系统架构

最终采用的方案是使用不同的 DDS Domain 隔离两台车。

```text
robot1: ROS_DOMAIN_ID=1
robot2: ROS_DOMAIN_ID=2
```

电脑端主要系统运行在 `ROS_DOMAIN_ID=1`：

```text
Gazebo: domain1
robot1 RViz: domain1
robot2 RViz: domain1
robot1 数字孪生绑定节点: domain1
robot2 数字孪生绑定节点: domain1
```

第二台车的数据通过电脑端桥接节点从 domain2 转发到 domain1，并统一加上 `/car2` 前缀。

整体结构如下：

```text
robot1, domain1
  /scan
  /odom
  /tf
  /tf_static
  /cmd_vel
      |
      v
电脑 domain1 直接使用


robot2, domain2
  /scan
  /odom
  /tf
  /tf_static
  /cmd_vel
      |
      v
domain bridge
      |
      v
电脑 domain1
  /car2/scan
  /car2/odom
  /car2/tf
  /car2/tf_static
  /car2/cmd_vel
```

这种设计的优点是：

- `robot1` 不需要改动。
- `robot2` 车端也可以保持原始 Nav2 和底盘配置。
- 电脑端通过桥接完成多车命名空间隔离。
- Gazebo 只需要运行在一个 domain 中。

## 4. 话题桥接设计

桥接节点位于：

```text
src/mycar_base_sim/scripts/domain_topic_bridge.py
```

启动文件位于：

```text
src/mycar_base_sim/app/car2_domain_bridge.launch.py
```

该桥接节点在同一个 Python 进程中创建两个 ROS 2 context：

- source context: `domain_id=2`
- target context: `domain_id=1`

然后进行话题转发。

### 4.1 robot2 到电脑端的正向桥接

robot2 原始话题在 domain2 中为：

```text
/scan
/odom
/odometry/filtered
/joint_states
/tf
/tf_static
/map
/map_updates
/amcl_pose
/particle_cloud
/robot_description
```

桥接后，在 domain1 中变为：

```text
/car2/scan
/car2/odom
/car2/odometry/filtered
/car2/joint_states
/car2/tf
/car2/tf_static
/car2/map
/car2/map_updates
/car2/amcl_pose
/car2/particle_cloud
/car2/robot_description
```

### 4.2 电脑端到 robot2 的反向桥接

为了让电脑端 RViz 可以控制 robot2，需要反向桥接：

```text
domain1 /car2/cmd_vel      -> domain2 /cmd_vel
domain1 /car2/initialpose  -> domain2 /initialpose
domain1 /car2/goal_pose    -> domain2 /goal_pose
```

这使得在 robot2 的 RViz 窗口中设置目标点时，不会影响 robot1。

## 5. Gazebo 数字孪生设计

Gazebo 中生成两台车：

```text
V550_akm_robot
V550_akm_robot_2
```

其中：

```text
V550_akm_robot   对应 robot1
V550_akm_robot_2 对应 robot2
```

### 5.1 robot1 的 Gazebo 绑定

robot1 的绑定方式已经验证可用。

其 Gazebo 位姿由真实车定位 TF 驱动：

```text
/tf 中 map -> base_footprint
```

绑定节点读取 TF 后调用 Gazebo 服务：

```text
/set_entity_state
```

将 Gazebo 中的 `V550_akm_robot` 设置到对应位置。

### 5.2 robot2 的 Gazebo 绑定

robot2 最终也采用与 robot1 相同的方式：

```text
/car2/tf 中 map -> base_footprint
```

即 robot2 的 Gazebo 模型也使用 AMCL/定位后的全局 TF，而不是直接使用 `/car2/odom`。

关键配置文件为：

```text
src/mycar_base_sim/config/sim_car_tf_twin_robot2.yaml
```

关键参数：

```yaml
pose_source: tf
global_frame: map
base_frame: base_footprint
cmd_vel_topic: /car2/cmd_vel
model_name: V550_akm_robot_2
set_entity_service: /set_entity_state
```

这样做的原因是：RViz 中的 `Set Initial Pose` 会改变 AMCL 产生的 `map -> odom_combined`，从而影响 `map -> base_footprint`。如果 Gazebo 跟随的是 TF，则 RViz 重新定位后 Gazebo 也会同步跳转。

如果 Gazebo 只跟随 `/car2/odom`，那么设置初始位姿不会改变 odom，Gazebo 也不会同步更新。

## 6. Gazebo 状态插件问题

一开始 Gazebo 中的小车无法跟随真实车运动。排查时发现：

```text
/car2/odom 有数据
/sim_car_tf_twin_robot2 存在
/sim_car_tf_twin_robot2 订阅了 /car2/odom
/set_entity_state 在 service list 中能看到
```

但实际调用：

```bash
ros2 service call /set_entity_state gazebo_msgs/srv/SetEntityState ...
```

会一直等待服务可用。

进一步检查 `/gazebo` 节点的 Service Servers 后发现，Gazebo 节点实际上没有提供 `/set_entity_state` 服务。

根本原因是 world 文件中没有加载：

```text
libgazebo_ros_state.so
```

因此在 Gazebo 启动时动态注入插件：

```xml
<plugin name="gazebo_ros_state" filename="libgazebo_ros_state.so">
  <update_rate>30.0</update_rate>
</plugin>
```

修改位置：

```text
src/wheeltec_robot_urdf/launch/display_gazebo.launch.py
```

该插件加载后，Gazebo 才真正提供：

```text
/get_entity_state
/set_entity_state
```

这是 Gazebo 数字孪生能否运动的关键。

## 7. RViz 可视化设计

系统中使用两个 RViz 配置文件：

```text
src/wheeltec_robot_urdf/rviz/car1.rviz
src/wheeltec_robot_urdf/rviz/car2.rviz
```

### 7.1 car1 RViz

car1 RViz 使用 robot1 原始话题：

```text
RobotModel: /robot_description
TF: /tf
LaserScan: /scan
Map: /map
InitialPose: /initialpose
GoalPose: /goal_pose
```

### 7.2 car2 RViz

car2 RViz 使用桥接后的话题：

```text
RobotModel: /car2/robot_description
TF: /car2/tf
LaserScan: /car2/scan
Map: /car2/map
AMCL Pose: /car2/amcl_pose
ParticleCloud: /car2/particle_cloud
InitialPose: /car2/initialpose
GoalPose: /car2/goal_pose
```

其中 `/car2/initialpose` 和 `/car2/goal_pose` 会由 bridge 转发回 domain2：

```text
/car2/initialpose -> /initialpose
/car2/goal_pose   -> /goal_pose
```

这样 car2 RViz 中的 2D Pose Estimate 和 2D Goal Pose 不会影响 robot1。

## 8. robot_description 与 TF 对齐问题

这是本项目中非常重要的一个问题。

robot1 的 RViz 模型能够正常显示，是因为：

```text
RobotModel 订阅: /robot_description
URDF link: base_link
TF frame: base_link
```

三者名称一致。

而 Gazebo 中第二个模型为了避免与第一个模型冲突，使用了加前缀的 URDF：

```text
/robot2/robot_description
```

其中 link 名类似：

```text
robot2_base_link
robot2_laser_link
```

但 robot2 实物车的 TF 仍然是：

```text
base_link
laser_link
base_footprint
```

因此，car2 RViz 不能直接使用：

```text
/robot2/robot_description
```

否则 RobotModel 会找不到对应 TF。

最终做法是额外发布一份未加 `robot2_` 前缀的 robot description：

```text
/car2/robot_description
```

它用于 car2 RViz 显示真实车模型。

而 `/robot2/robot_description` 仍然保留给 Gazebo 中的第二个仿真模型使用。

最终关系为：

```text
/robot_description       -> robot1 RViz / Gazebo robot1
/car2/robot_description  -> car2 RViz，未加前缀，匹配真实 car2 TF
/robot2/robot_description -> Gazebo robot2，加 robot2_ 前缀
```

## 9. 典型问题与解决过程

### 9.1 两台车同时连接失败

现象：

两台车同时接入时，话题名称冲突，TF 和 scan 数据混杂。

原因：

两台车都在同一个 ROS Domain 中发布相同的话题名称。

解决：

使用不同 domain：

```text
robot1 -> ROS_DOMAIN_ID=1
robot2 -> ROS_DOMAIN_ID=2
```

并用桥接节点把 robot2 数据转发为 `/car2/...`。

### 9.2 car2 单独运行 Nav2 报 TF extrapolation

曾出现错误：

```text
Lookup would require extrapolation into the past
```

这类问题通常和以下因素有关：

- 机器时间不同步。
- `use_sim_time` 设置错误。
- scan、odom、tf 时间戳不一致。
- 雷达数据延迟。
- AMCL 发布 TF 的时间落后于 Nav2 请求时间。

后来确认 robot2 单独运行 Nav2 可以正常工作，说明主要问题不在车端，而在多车重映射和桥接层。

### 9.3 雷达 CRC 报错

曾出现：

```text
CRC check failed
Abandon the current data packet
```

这通常说明雷达数据包校验失败，可能原因包括：

- 线缆接触不良。
- 雷达供电不稳定。
- 串口/网口参数不匹配。
- 多个驱动同时读取同一个雷达。
- 网络包丢失。

该错误会影响 scan 稳定性，但与 Gazebo 绑定逻辑不是同一个问题。

### 9.4 Gazebo 里的车不动

最初现象：

RViz 中车动，但 Gazebo 中车不动。

排查发现：

```text
/car2/odom 有数据
/sim_car_tf_twin_robot2 存在
/sim_car_tf_twin_robot2 订阅了 /car2/odom
```

但 `/set_entity_state` 实际不可调用。

原因：

Gazebo world 没有加载 `libgazebo_ros_state.so`。

解决：

在 world 中注入 `gazebo_ros_state` 插件。

### 9.5 Gazebo 方向反了

robot2 使用 odom 驱动时，曾出现 Gazebo 中前后方向与真实车相反的问题。

临时解决方式是调整 yaw offset。

后来 robot2 改为使用 `map -> base_footprint` TF 驱动，offset 归零，方向问题随 TF 统一处理。

### 9.6 car2 RViz 中模型不显示

原因：

car2 RViz 误用了 `/robot2/robot_description`，该 URDF 的 link 名带 `robot2_` 前缀，无法匹配 car2 的真实 TF。

解决：

使用 `/car2/robot_description`，其 link 名保持：

```text
base_link
laser_link
base_footprint
```

与 `/car2/tf` 中的 frame 对齐。

### 9.7 car2 RViz 的 goal 会影响 robot1

原因：

RViz 默认工具发布到全局：

```text
/initialpose
/goal_pose
```

如果 car2 RViz 也使用这些话题，就会影响 robot1。

解决：

car2 RViz 改为：

```text
/car2/initialpose
/car2/goal_pose
```

再由 bridge 转发回 domain2 的原始话题。

### 9.8 car1 RViz 中出现死的 car2 TF

原因：

Gazebo launch 中曾手动发布静态 TF：

```text
map -> robot2_base_footprint
robot2_base_footprint -> robot2_base_link
```

这些 TF 与真实 car2 无关，所以在 car1 RViz 中表现为一个不动的 car2 TF。

解决：

删除这些静态 TF。

## 10. 当前启动流程

### 10.1 robot1

robot1 使用原始启动方式，运行在：

```bash
export ROS_DOMAIN_ID=1
```

### 10.2 robot2

robot2 使用原始启动方式，运行在：

```bash
export ROS_DOMAIN_ID=2
```

robot2 车端不需要改成 `/car2/...`，仍然使用原始话题。

### 10.3 电脑端桥接

```bash
source /home/j/桌面/work/install/setup.bash
ros2 launch mycar_base_sim car2_domain_bridge.launch.py
```

### 10.4 Gazebo 与双 RViz

```bash
source /home/j/桌面/work/install/setup.bash
export ROS_DOMAIN_ID=1
ros2 launch mycar_sim_bringup sim_bringup_two_cars.launch.py
```

如果只需要 Gazebo，不需要 RViz：

```bash
ros2 launch mycar_sim_bringup sim_bringup_two_cars_no_rviz.launch.py
```

## 11. 常用检查命令

检查 car2 桥接后的话题：

```bash
export ROS_DOMAIN_ID=1
ros2 topic list | grep car2
```

检查 car2 scan：

```bash
ros2 topic echo --once /car2/scan
```

检查 car2 odom：

```bash
ros2 topic echo --once /car2/odom
```

检查 car2 TF：

```bash
ros2 run tf2_ros tf2_echo map base_footprint \
  --ros-args -r /tf:=/car2/tf -r /tf_static:=/car2/tf_static
```

检查 Gazebo 状态服务：

```bash
ros2 service list | grep entity
```

应包含：

```text
/get_entity_state
/set_entity_state
```

检查 Gazebo 中模型列表：

```bash
ros2 service call /get_model_list gazebo_msgs/srv/GetModelList "{}"
```

应包含：

```text
V550_akm_robot
V550_akm_robot_2
```

检查 robot2 数字孪生绑定节点：

```bash
ros2 param get /sim_car_tf_twin_robot2 pose_source
ros2 param get /sim_car_tf_twin_robot2 model_name
```

期望输出：

```text
pose_source: tf
model_name: V550_akm_robot_2
```

## 12. 项目经验总结

这个项目的核心难点不是单个模块，而是多个 ROS 2 子系统之间的坐标系、话题、服务和 domain 的一致性。

需要同时处理：

```text
DDS Domain 隔离
话题桥接
TF 树一致性
robot_description 与 TF frame 对齐
Gazebo /set_entity_state 服务
RViz 工具话题隔离
AMCL 定位与 Gazebo 坐标同步
```

最终方案的关键思想是：

1. 不改动 robot1。
2. 不强行修改 robot2 车端话题。
3. 用不同 ROS_DOMAIN_ID 隔离两台车。
4. 在电脑端将 robot2 数据桥接为 `/car2/...`。
5. Gazebo 统一运行在 domain1。
6. 两台 Gazebo 模型都由 `map -> base_footprint` TF 驱动。
7. RViz 中每台车只操作自己的话题。

这种架构具有较好的可扩展性。后续如果增加第三台车，也可以采用类似方式：

```text
robot3 domain3 -> bridge -> /car3/...
```

但需要注意，随着车辆数量增加，TF、map、goal、initialpose、cmd_vel 等话题都必须严格区分，否则会出现跨车控制和显示污染问题。

## 13. robot2 静止时 Gazebo 小车抖动/缓慢运动问题

在后续测试中，robot2 的 Gazebo 数字孪生模型虽然已经可以跟随真实车运动，但出现了一个新现象：

```text
真实车静止时，Gazebo 中的 robot2 仍然会小幅运动或缓慢转动。
```

最初怀疑是 Gazebo 绑定节点滤波不足，后来通过 TF 排查发现，问题主要来自 robot2 自身定位链路的缓慢漂移。

### 13.1 现象

在 car2 RViz 中，robot2 的定位可以显示；Gazebo 中 `V550_akm_robot_2` 也能跟随 car2 的定位移动。

但是当真实 robot2 静止时，Gazebo 中的模型仍会间歇性移动或缓慢旋转。

这说明 Gazebo 并不是随机运动，而是在跟随某个持续变化的输入。

### 13.2 当前 Gazebo robot2 的驱动源

robot2 的 Gazebo 绑定节点最终配置为：

```yaml
pose_source: tf
global_frame: map
base_frame: base_footprint
model_name: V550_akm_robot_2
```

因此 Gazebo 中 robot2 的位姿来源是：

```text
/car2/tf 中的 map -> base_footprint
```

这与 robot1 的实现保持一致。

### 13.3 TF 排查方法

为了区分是 AMCL 漂移还是 odom/IMU 漂移，分别检查两段 TF：

```bash
ros2 run tf2_ros tf2_echo map odom_combined \
  --ros-args -r /tf:=/car2/tf -r /tf_static:=/car2/tf_static
```

以及：

```bash
ros2 run tf2_ros tf2_echo odom_combined base_footprint \
  --ros-args -r /tf:=/car2/tf -r /tf_static:=/car2/tf_static
```

如果第一条 `map -> odom_combined` 在变，而第二条基本稳定，则主要是 AMCL 在不断修正定位。

如果第二条 `odom_combined -> base_footprint` 自身也在变化，则说明 odom/EKF/IMU 链路本身存在漂移。

### 13.4 实际排查结果

实际测试中，`map -> odom_combined` 存在变化，例如：

```text
Translation: [0.092, 0.455, 0.000]
Translation: [0.049, 0.450, 0.000]
Translation: [0.019, 0.444, 0.000]
Translation: [-0.011, 0.442, 0.000]
```

这说明 AMCL 在持续修正 `map -> odom_combined`。

进一步检查 `odom_combined -> base_footprint` 时发现，即使位置基本不变，yaw 也在持续变化：

```text
-90.984 deg
-91.568 deg
-92.120 deg
-93.304 deg
-95.060 deg
-97.982 deg
-100.919 deg
-103.279 deg
```

这说明问题不只是 AMCL，robot2 的 odom/IMU/EKF 链路本身也存在角度漂移。

### 13.5 IMU 检查

在 robot2 原始 domain 中检查 IMU：

```bash
export ROS_DOMAIN_ID=2
ros2 topic echo --once /imu/data_raw
```

观察到静止时 IMU z 轴角速度存在偏置：

```text
angular_velocity:
  z: -0.005595240276306868
```

该值看起来不大，但如果 EKF 或里程计融合持续积分该角速度，就会导致 yaw 缓慢漂移。

链路可以概括为：

```text
IMU z 轴角速度偏置
  -> odom / EKF yaw 缓慢漂移
  -> AMCL 持续修正 map -> odom_combined
  -> map -> base_footprint 变化
  -> Gazebo 中 robot2 跟随变化
```

### 13.6 AMCL 参数调整建议

robot2 的 Nav2 参数中，AMCL 原始配置较敏感：

```yaml
amcl:
  ros__parameters:
    update_min_a: 0.03
    update_min_d: 0.03
    resample_interval: 1
    transform_tolerance: 1.0
```

其中：

- `update_min_a` 表示触发 AMCL 更新所需的最小旋转量。
- `update_min_d` 表示触发 AMCL 更新所需的最小平移量。
- `resample_interval` 表示多少次滤波更新后进行一次重采样。
- `transform_tolerance` 是 TF 发布容差。

当 `update_min_a` 和 `update_min_d` 太小时，静止状态下的轻微 odom/IMU 漂移也会触发 AMCL 更新。

建议先改为：

```yaml
amcl:
  ros__parameters:
    transform_tolerance: 0.5
    update_min_a: 0.10
    update_min_d: 0.10
    resample_interval: 2
```

如果仍然抖动，可以进一步放宽为：

```yaml
amcl:
  ros__parameters:
    update_min_a: 0.15
    update_min_d: 0.15
    resample_interval: 3
```

但需要注意，阈值过大会降低 AMCL 对真实运动的响应速度。

### 13.7 use_sim_time 配置一致性

在 robot2 的 Nav2 参数中，大部分节点配置为：

```yaml
use_sim_time: False
```

但曾发现：

```yaml
smoother_server:
  ros__parameters:
    use_sim_time: True
```

实车系统中建议统一使用：

```yaml
use_sim_time: False
```

因此应改为：

```yaml
smoother_server:
  ros__parameters:
    use_sim_time: False
```

虽然该项不是 robot2 静止漂移的主因，但时间源不一致会增加 Nav2 行为不确定性。

### 13.8 数字孪生显示层的短期处理

如果短期内无法完全解决 IMU/EKF 漂移，可以在 Gazebo 绑定层加入 deadband。

思路是：如果新位姿与上一次发布给 Gazebo 的位姿差异很小，就不调用 `/set_entity_state`。

例如：

```yaml
pose_deadband: 0.03
yaw_deadband: 0.05
```

含义：

```text
位置变化小于 0.03 m 时忽略
角度变化小于 0.05 rad 时忽略
```

这样可以过滤静止时的小幅 AMCL/odom 抖动，同时真实车明显运动时 Gazebo 仍会跟随。

不过该方法只是显示层滤波，不解决定位源头问题。

### 13.9 根本解决方向

如果希望从根本上解决该问题，应继续检查 robot2 车端：

1. IMU 是否完成静态校准。
2. IMU z 轴角速度是否存在固定偏置。
3. EKF 是否过度信任 IMU yaw 或 yaw rate。
4. 底盘编码器是否在静止时仍输出非零速度。
5. 是否需要加入静止检测或零速约束。
6. AMCL 初始位姿是否准确。
7. 地图与实际环境是否一致。
8. 雷达外参与 `base_link` 是否准确。

### 13.10 本问题的结论

robot2 Gazebo 模型静止时仍缓慢运动的主要原因不是 Gazebo，也不是 bridge，而是 robot2 的定位输入本身在变化。

当前观察结果表明：

```text
odom_combined -> base_footprint 的 yaw 在静止时持续变化
IMU angular_velocity.z 存在小偏置
AMCL map -> odom_combined 也在持续修正
```

因此应同时从两层处理：

```text
车端定位层：校准 IMU / 调整 EKF / 调整 AMCL 参数
显示绑定层：增加滤波或 deadband，避免 Gazebo 视觉抖动
```
