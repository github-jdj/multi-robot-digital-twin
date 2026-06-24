cmake_minimum_required(VERSION 3.8)
project(mycar_base_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

include_directories(include)

add_executable(sim_car_tf_twin_node
  src/sim_car_tf_twin.cpp
)

ament_target_dependencies(sim_car_tf_twin_node
  rclcpp
  geometry_msgs
  nav_msgs
  std_msgs
  gazebo_msgs
  tf2
  tf2_ros
)

install(
  TARGETS
    sim_car_tf_twin_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/domain_topic_bridge.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
    app
    config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
