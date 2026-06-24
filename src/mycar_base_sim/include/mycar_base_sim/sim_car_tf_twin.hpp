#ifndef MYCAR_BASE_SIM_SIM_CAR_TF_TWIN_HPP_
#define MYCAR_BASE_SIM_SIM_CAR_TF_TWIN_HPP_

#include <memory>
#include <string>

#include "gazebo_msgs/msg/entity_state.hpp"
#include "gazebo_msgs/srv/set_entity_state.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace mycar_base_sim
{

class SimCarTfTwin : public rclcpp::Node
{
public:
  SimCarTfTwin();

private:
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void timerCallback();

  double clamp(double value, double lower, double upper) const;
  double normalizeAngle(double angle) const;
  double yawFromQuaternion(const geometry_msgs::msg::Quaternion & q) const;
  geometry_msgs::msg::Quaternion quaternionFromYaw(double yaw) const;

  double rightFrontAngleFromCurvature(double curvature) const;
  double leftFrontAngleFromCurvature(double curvature) const;

  bool lookupTfPose(double & x, double & y, double & yaw);
  bool lookupOdomPose(double & x, double & y, double & yaw);
  void resetFilter(double x, double y, double yaw);
  void updateFilteredPose(double x, double y, double yaw);

  void setGazeboPose(double x, double y, double z, double yaw);
  void publishWheelAnimation(double vehicle_speed, double yaw_rate);

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr rear_wheel_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr steering_pub_;
  rclcpp::Client<gazebo_msgs::srv::SetEntityState>::SharedPtr set_entity_client_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::string global_frame_;
  std::string base_frame_;
  std::string pose_source_;
  std::string odom_topic_;
  std::string cmd_vel_topic_;
  std::string set_entity_service_;
  std::string model_name_;
  std::string reference_frame_;
  std::string rear_wheel_command_topic_;
  std::string steering_command_topic_;

  double max_linear_speed_;
  double max_right_front_steering_angle_;
  double max_left_front_steering_angle_;
  double wheel_base_;
  double track_width_;
  double wheel_radius_;
  double z_offset_;
  double gazebo_x_offset_;
  double gazebo_y_offset_;
  double gazebo_yaw_offset_;
  double publish_rate_;
  double cmd_vel_timeout_sec_;

  double pose_filter_alpha_;
  double yaw_filter_alpha_;
  double reset_jump_distance_;
  double reset_jump_yaw_;

  bool invert_rear_wheel_speed_;

  bool has_odom_pose_;
  double odom_x_;
  double odom_y_;
  double odom_yaw_;

  bool has_filtered_pose_;
  double filtered_x_;
  double filtered_y_;
  double filtered_yaw_;

  geometry_msgs::msg::Twist last_cmd_vel_;
  rclcpp::Time last_cmd_vel_time_;
};

}  // namespace mycar_base_sim

#endif  // MYCAR_BASE_SIM_SIM_CAR_TF_TWIN_HPP_
