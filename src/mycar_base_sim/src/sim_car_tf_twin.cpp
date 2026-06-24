#include "mycar_base_sim/sim_car_tf_twin.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>

#include "tf2/exceptions.h"

namespace mycar_base_sim
{

SimCarTfTwin::SimCarTfTwin()
: Node("sim_car_tf_twin"),
  max_linear_speed_(1.2),
  max_right_front_steering_angle_(0.5),
  max_left_front_steering_angle_(0.5),
  wheel_base_(0.143),
  track_width_(0.162),
  wheel_radius_(0.032),
  z_offset_(0.08),
  gazebo_x_offset_(0.0),
  gazebo_y_offset_(0.0),
  gazebo_yaw_offset_(0.0),
  publish_rate_(30.0),
  cmd_vel_timeout_sec_(0.5),
  pose_filter_alpha_(0.12),
  yaw_filter_alpha_(0.12),
  reset_jump_distance_(0.8),
  reset_jump_yaw_(1.2),
  invert_rear_wheel_speed_(true),
  has_odom_pose_(false),
  odom_x_(0.0),
  odom_y_(0.0),
  odom_yaw_(0.0),
  has_filtered_pose_(false),
  filtered_x_(0.0),
  filtered_y_(0.0),
  filtered_yaw_(0.0)
{
  this->declare_parameter<std::string>("global_frame", "map");
  this->declare_parameter<std::string>("base_frame", "base_footprint");
  this->declare_parameter<std::string>("pose_source", "tf");
  this->declare_parameter<std::string>("odom_topic", "/odom");
  this->declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");

  this->declare_parameter<std::string>("set_entity_service", "/set_entity_state");
  this->declare_parameter<std::string>("model_name", "V550_akm_robot");
  this->declare_parameter<std::string>("reference_frame", "world");

  this->declare_parameter<std::string>(
    "rear_wheel_command_topic",
    "/rear_wheel_velocity_controller/commands"
  );
  this->declare_parameter<std::string>(
    "steering_command_topic",
    "/steering_position_controller/commands"
  );

  this->declare_parameter<double>("max_linear_speed", 1.2);
  this->declare_parameter<double>("max_right_front_steering_angle", 0.5);
  this->declare_parameter<double>("max_left_front_steering_angle", 0.5);

  this->declare_parameter<double>("wheel_base", 0.143);
  this->declare_parameter<double>("track_width", 0.162);
  this->declare_parameter<double>("wheel_radius", 0.032);
  this->declare_parameter<double>("z_offset", 0.08);
  this->declare_parameter<double>("gazebo_x_offset", 0.0);
  this->declare_parameter<double>("gazebo_y_offset", 0.0);
  this->declare_parameter<double>("gazebo_yaw_offset", 0.0);

  this->declare_parameter<double>("publish_rate", 30.0);
  this->declare_parameter<double>("cmd_vel_timeout_sec", 0.5);

  this->declare_parameter<double>("pose_filter_alpha", 0.12);
  this->declare_parameter<double>("yaw_filter_alpha", 0.12);
  this->declare_parameter<double>("reset_jump_distance", 0.8);
  this->declare_parameter<double>("reset_jump_yaw", 1.2);

  this->declare_parameter<bool>("invert_rear_wheel_speed", true);

  global_frame_ = this->get_parameter("global_frame").as_string();
  base_frame_ = this->get_parameter("base_frame").as_string();
  pose_source_ = this->get_parameter("pose_source").as_string();
  odom_topic_ = this->get_parameter("odom_topic").as_string();
  cmd_vel_topic_ = this->get_parameter("cmd_vel_topic").as_string();

  set_entity_service_ = this->get_parameter("set_entity_service").as_string();
  model_name_ = this->get_parameter("model_name").as_string();
  reference_frame_ = this->get_parameter("reference_frame").as_string();

  rear_wheel_command_topic_ = this->get_parameter("rear_wheel_command_topic").as_string();
  steering_command_topic_ = this->get_parameter("steering_command_topic").as_string();

  max_linear_speed_ = this->get_parameter("max_linear_speed").as_double();
  max_right_front_steering_angle_ =
    this->get_parameter("max_right_front_steering_angle").as_double();
  max_left_front_steering_angle_ =
    this->get_parameter("max_left_front_steering_angle").as_double();

  wheel_base_ = this->get_parameter("wheel_base").as_double();
  track_width_ = this->get_parameter("track_width").as_double();
  wheel_radius_ = this->get_parameter("wheel_radius").as_double();
  z_offset_ = this->get_parameter("z_offset").as_double();
  gazebo_x_offset_ = this->get_parameter("gazebo_x_offset").as_double();
  gazebo_y_offset_ = this->get_parameter("gazebo_y_offset").as_double();
  gazebo_yaw_offset_ = this->get_parameter("gazebo_yaw_offset").as_double();

  publish_rate_ = this->get_parameter("publish_rate").as_double();
  cmd_vel_timeout_sec_ = this->get_parameter("cmd_vel_timeout_sec").as_double();

  pose_filter_alpha_ = clamp(this->get_parameter("pose_filter_alpha").as_double(), 0.01, 1.0);
  yaw_filter_alpha_ = clamp(this->get_parameter("yaw_filter_alpha").as_double(), 0.01, 1.0);
  reset_jump_distance_ = std::max(0.0, this->get_parameter("reset_jump_distance").as_double());
  reset_jump_yaw_ = std::max(0.0, this->get_parameter("reset_jump_yaw").as_double());

  invert_rear_wheel_speed_ = this->get_parameter("invert_rear_wheel_speed").as_bool();

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    cmd_vel_topic_,
    10,
    std::bind(&SimCarTfTwin::cmdVelCallback, this, std::placeholders::_1)
  );

  if (pose_source_ == "odom") {
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_,
      10,
      std::bind(&SimCarTfTwin::odomCallback, this, std::placeholders::_1)
    );
  }

  rear_wheel_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
    rear_wheel_command_topic_,
    10
  );

  steering_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
    steering_command_topic_,
    10
  );

  set_entity_client_ = this->create_client<gazebo_msgs::srv::SetEntityState>(
    set_entity_service_
  );

  last_cmd_vel_time_ = this->now();

  const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, publish_rate_));
  timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&SimCarTfTwin::timerCallback, this)
  );

  RCLCPP_INFO(this->get_logger(), "sim_car_tf_twin started");
  if (pose_source_ == "odom") {
    RCLCPP_INFO(this->get_logger(), "Pose source: Odometry %s", odom_topic_.c_str());
  } else {
    RCLCPP_INFO(this->get_logger(), "Pose source: TF %s -> %s", global_frame_.c_str(), base_frame_.c_str());
  }
  RCLCPP_INFO(this->get_logger(), "Wheel animation source: %s", cmd_vel_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "Gazebo service: %s", set_entity_service_.c_str());
  RCLCPP_INFO(
    this->get_logger(),
    "Smoothing: pose alpha %.3f, yaw alpha %.3f, publish %.1f Hz",
    pose_filter_alpha_,
    yaw_filter_alpha_,
    publish_rate_
  );
}

double SimCarTfTwin::clamp(double value, double lower, double upper) const
{
  return std::max(lower, std::min(value, upper));
}

double SimCarTfTwin::normalizeAngle(double angle) const
{
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }

  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }

  return angle;
}

double SimCarTfTwin::yawFromQuaternion(const geometry_msgs::msg::Quaternion & q) const
{
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

geometry_msgs::msg::Quaternion SimCarTfTwin::quaternionFromYaw(double yaw) const
{
  geometry_msgs::msg::Quaternion q;
  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(yaw * 0.5);
  q.w = std::cos(yaw * 0.5);
  return q;
}

double SimCarTfTwin::rightFrontAngleFromCurvature(double curvature) const
{
  const double half_track = track_width_ * 0.5;
  const double denominator = 1.0 + half_track * curvature;

  if (std::abs(denominator) < 1e-6) {
    return (curvature >= 0.0) ?
      max_right_front_steering_angle_ :
      -max_right_front_steering_angle_;
  }

  return std::atan((wheel_base_ * curvature) / denominator);
}

double SimCarTfTwin::leftFrontAngleFromCurvature(double curvature) const
{
  const double half_track = track_width_ * 0.5;
  const double denominator = 1.0 - half_track * curvature;

  if (std::abs(denominator) < 1e-6) {
    return (curvature >= 0.0) ?
      max_left_front_steering_angle_ :
      -max_left_front_steering_angle_;
  }

  return std::atan((wheel_base_ * curvature) / denominator);
}

void SimCarTfTwin::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  last_cmd_vel_ = *msg;
  last_cmd_vel_time_ = this->now();
}

void SimCarTfTwin::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  odom_x_ = msg->pose.pose.position.x;
  odom_y_ = msg->pose.pose.position.y;
  odom_yaw_ = yawFromQuaternion(msg->pose.pose.orientation);
  has_odom_pose_ = true;
}

bool SimCarTfTwin::lookupTfPose(double & x, double & y, double & yaw)
{
  try {
    const auto transform = tf_buffer_->lookupTransform(
      global_frame_,
      base_frame_,
      tf2::TimePointZero
    );

    x = transform.transform.translation.x;
    y = transform.transform.translation.y;
    yaw = yawFromQuaternion(transform.transform.rotation);
    return true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      2000,
      "TF lookup failed: %s",
      ex.what()
    );
    return false;
  }
}

bool SimCarTfTwin::lookupOdomPose(double & x, double & y, double & yaw)
{
  if (!has_odom_pose_) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      2000,
      "Waiting for odometry: %s",
      odom_topic_.c_str()
    );
    return false;
  }

  x = odom_x_;
  y = odom_y_;
  yaw = odom_yaw_;
  return true;
}

void SimCarTfTwin::resetFilter(double x, double y, double yaw)
{
  filtered_x_ = x;
  filtered_y_ = y;
  filtered_yaw_ = yaw;
  has_filtered_pose_ = true;
}

void SimCarTfTwin::updateFilteredPose(double x, double y, double yaw)
{
  if (!has_filtered_pose_) {
    resetFilter(x, y, yaw);
    return;
  }

  const double dx = x - filtered_x_;
  const double dy = y - filtered_y_;
  const double distance = std::hypot(dx, dy);
  const double yaw_error_for_reset = std::abs(normalizeAngle(yaw - filtered_yaw_));

  if (
    (reset_jump_distance_ > 0.0 && distance > reset_jump_distance_) ||
    (reset_jump_yaw_ > 0.0 && yaw_error_for_reset > reset_jump_yaw_)
  ) {
    resetFilter(x, y, yaw);
    return;
  }

  filtered_x_ += pose_filter_alpha_ * dx;
  filtered_y_ += pose_filter_alpha_ * dy;

  const double yaw_error = normalizeAngle(yaw - filtered_yaw_);
  filtered_yaw_ = normalizeAngle(filtered_yaw_ + yaw_filter_alpha_ * yaw_error);
}

void SimCarTfTwin::setGazeboPose(double x, double y, double z, double yaw)
{
  if (!set_entity_client_->service_is_ready()) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      2000,
      "Waiting for service: %s",
      set_entity_service_.c_str()
    );
    return;
  }

  gazebo_msgs::msg::EntityState state;
  state.name = model_name_;
  state.reference_frame = reference_frame_;

  state.pose.position.x = x;
  state.pose.position.y = y;
  state.pose.position.z = z;
  state.pose.orientation = quaternionFromYaw(yaw);

  // Digital twin mode: pose is authoritative. Keep Gazebo velocity zero to reduce physical jitter.
  state.twist.linear.x = 0.0;
  state.twist.linear.y = 0.0;
  state.twist.linear.z = 0.0;
  state.twist.angular.x = 0.0;
  state.twist.angular.y = 0.0;
  state.twist.angular.z = 0.0;

  auto request = std::make_shared<gazebo_msgs::srv::SetEntityState::Request>();
  request->state = state;

  set_entity_client_->async_send_request(request);
}

void SimCarTfTwin::publishWheelAnimation(double vehicle_speed, double yaw_rate)
{
  double curvature = 0.0;

  if (std::abs(vehicle_speed) >= 1e-4) {
    curvature = yaw_rate / vehicle_speed;
  }

  double right_front_angle = rightFrontAngleFromCurvature(curvature);
  double left_front_angle = leftFrontAngleFromCurvature(curvature);

  right_front_angle = clamp(
    right_front_angle,
    -max_right_front_steering_angle_,
    max_right_front_steering_angle_
  );

  left_front_angle = clamp(
    left_front_angle,
    -max_left_front_steering_angle_,
    max_left_front_steering_angle_
  );

  const double half_track = track_width_ * 0.5;

  double rear_left_linear_speed = 0.0;
  double rear_right_linear_speed = 0.0;

  if (std::abs(vehicle_speed) >= 1e-4) {
    rear_left_linear_speed = vehicle_speed * (1.0 - half_track * curvature);
    rear_right_linear_speed = vehicle_speed * (1.0 + half_track * curvature);
  }

  double rear_left_wheel_speed = rear_left_linear_speed / wheel_radius_;
  double rear_right_wheel_speed = rear_right_linear_speed / wheel_radius_;

  if (invert_rear_wheel_speed_) {
    rear_left_wheel_speed = -rear_left_wheel_speed;
    rear_right_wheel_speed = -rear_right_wheel_speed;
  }

  std_msgs::msg::Float64MultiArray rear_wheel_msg;
  rear_wheel_msg.data = {
    rear_left_wheel_speed,
    rear_right_wheel_speed
  };

  std_msgs::msg::Float64MultiArray steering_msg;
  steering_msg.data = {
    left_front_angle,
    right_front_angle
  };

  rear_wheel_pub_->publish(rear_wheel_msg);
  steering_pub_->publish(steering_msg);
}

void SimCarTfTwin::timerCallback()
{
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;

  const bool has_pose = (pose_source_ == "odom") ?
    lookupOdomPose(x, y, yaw) :
    lookupTfPose(x, y, yaw);

  if (!has_pose) {
    publishWheelAnimation(0.0, 0.0);
    return;
  }

  updateFilteredPose(x, y, yaw);

  double vehicle_speed = 0.0;
  double yaw_rate = 0.0;

  const double cmd_age = (this->now() - last_cmd_vel_time_).seconds();

  if (cmd_age <= cmd_vel_timeout_sec_) {
    vehicle_speed = clamp(last_cmd_vel_.linear.x, -max_linear_speed_, max_linear_speed_);
    yaw_rate = last_cmd_vel_.angular.z;
  }

  setGazeboPose(
    gazebo_x_offset_ + filtered_x_,
    gazebo_y_offset_ + filtered_y_,
    z_offset_,
    normalizeAngle(gazebo_yaw_offset_ + filtered_yaw_)
  );
  publishWheelAnimation(vehicle_speed, yaw_rate);
}

}  // namespace mycar_base_sim

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  const auto node = std::make_shared<mycar_base_sim::SimCarTfTwin>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
