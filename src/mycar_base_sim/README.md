# mycar_base_sim

Digital twin base package without joystick.

## What it does

The Gazebo model pose is driven by localization TF:

```text
map -> base_footprint
```

The node calls:

```text
/set_entity_state
```

to set the Gazebo model pose.

Wheel animation uses:

```text
/cmd_vel.linear.x
/cmd_vel.angular.z
```

and publishes:

```text
/rear_wheel_velocity_controller/commands
/steering_position_controller/commands
```

## No joystick

This version has no `joy_node`, no mode switch, and no remote-control branch.

## Smoothing

Pose smoothing is configured in:

```text
config/sim_car_tf_twin.yaml
```

Default:

```yaml
publish_rate: 30.0
pose_filter_alpha: 0.12
yaw_filter_alpha: 0.12
```

Smaller alpha means smoother but more delay.

## Build

```bash
cd ~/workspace/ws_sim
colcon build --packages-select mycar_base_sim
source install/setup.bash
```

## Run

Start Gazebo first:

```bash
ros2 launch wheeltec_robot_urdf display_gazebo.launch.py
```

Start digital twin base:

```bash
ros2 launch mycar_base_sim base_sim_twin.launch.py
```

## Check

```bash
ros2 run tf2_ros tf2_echo map base_footprint
ros2 service list | grep entity
```

The service should be:

```text
/set_entity_state
```

If your service is `/gazebo/set_entity_state`, edit:

```yaml
set_entity_service: /gazebo/set_entity_state
```
