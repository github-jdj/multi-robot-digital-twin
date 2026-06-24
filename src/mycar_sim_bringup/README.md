# mycar_sim_bringup

`ament_cmake` bringup package with built-in `ROS_DOMAIN_ID` setup.

## Starts

```text
wheeltec_robot_urdf/launch/display_gazebo.launch.py
mycar_base_sim/app/base_sim_twin.launch.py
wheeltec_robot_urdf/rviz/default.rviz
```

## Default domain

The launch file sets:

```text
ROS_DOMAIN_ID=1
```

You can override it:

```bash
ros2 launch mycar_sim_bringup sim_bringup.launch.py domain_id:=0
```

## Build

```bash
cd /home/baater/workspace/ws_sim
colcon build --packages-select mycar_sim_bringup
source install/setup.bash
```

## Launch with RViz

```bash
ros2 launch mycar_sim_bringup sim_bringup.launch.py
```

## Launch without RViz

```bash
ros2 launch mycar_sim_bringup sim_bringup_no_rviz.launch.py
```
