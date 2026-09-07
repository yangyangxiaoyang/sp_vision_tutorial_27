#include "sp_controller_server/controller_server.hpp"

#include <algorithm>
#include <cmath>

#include <tf2/exceptions.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace sp_controller_server {

ControllerServer::ControllerServer()
: Node("controller_server"),
  loader_("sp_controller_server", "sp_controller_server::ControllerPlugin")
{
  local_path_topic_ = this->declare_parameter<std::string>("local_path_topic", "/local_path");
  odom_topic_       = this->declare_parameter<std::string>("odom_topic", "/Odometry");
  odom_frame_id_    = this->declare_parameter<std::string>("odom_frame_id", "lidar_odom");
  map_frame_id_     = this->declare_parameter<std::string>("map_frame_id", "map");
  cmd_vel_topic_    = this->declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
  plugin_name_      = this->declare_parameter<std::string>("plugin_name", "PidController");
  plugin_type_      = this->declare_parameter<std::string>("plugin_type", "PidController");
  base_frame_id_    = this->declare_parameter<std::string>("base_frame_id", "base_link");
  control_frequency_= this->declare_parameter<double>("control_frequency", 20.0);
  speed_limit_      = this->declare_parameter<double>("speed_limit", 0.5);

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  local_path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
    local_path_topic_, 10,
    std::bind(&ControllerServer::onLocalPath, this, std::placeholders::_1));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    odom_topic_, rclcpp::SensorDataQoS().keep_last(1),
    std::bind(&ControllerServer::onOdometry, this, std::placeholders::_1));

  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);

  set_control_enable_srv_ = this->create_service<std_srvs::srv::SetBool>(
    "/set_control_enable",
    std::bind(&ControllerServer::onSetControlEnable, this,
      std::placeholders::_1, std::placeholders::_2));

  if (control_frequency_ <= 0.0) {
    RCLCPP_WARN(this->get_logger(), "control_frequency must be positive. Resetting to 10 Hz.");
    control_frequency_ = 10.0;
  }

  control_timer_ = this->create_wall_timer(
    std::chrono::duration<double>(1.0 / control_frequency_),
    std::bind(&ControllerServer::controlLoop, this));
}

void ControllerServer::init()
{
  loadPlugin();
}

void ControllerServer::loadPlugin()
{
  try {
    controller_ = loader_.createSharedInstance(plugin_type_);
    rclcpp::Node::SharedPtr node_ptr(this, [](rclcpp::Node *) {});
    controller_->configure(node_ptr, plugin_name_, tf_buffer_);
    controller_->setSpeedLimit(speed_limit_);
    RCLCPP_INFO(this->get_logger(), "Loaded controller plugin: %s (%s)",
      plugin_name_.c_str(), plugin_type_.c_str());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(this->get_logger(), "Failed to load controller plugin: %s", e.what());
    throw;
  }
}

void ControllerServer::onLocalPath(const nav_msgs::msg::Path::SharedPtr msg)
{
  nav_msgs::msg::Path processed_path = *msg;

  const bool has_embedded_velocity = std::any_of(
    processed_path.poses.begin(), processed_path.poses.end(),
    [](const geometry_msgs::msg::PoseStamped & pose) {
      return std::fabs(pose.pose.orientation.x) > 1e-4 ||
             std::fabs(pose.pose.orientation.y) > 1e-4;
    });

  if (controller_) {
    controller_->setPlan(processed_path);
  }

  {
    std::lock_guard<std::mutex> lock(path_mutex_);
    latest_path_ = std::move(processed_path);
  }

  if (has_embedded_velocity) {
    RCLCPP_DEBUG(this->get_logger(),
      "Local path contains embedded velocity profile in pose.orientation.x/y");
  }
}

void ControllerServer::onOdometry(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(velocity_mutex_);

  // 保存原始里程计速度（里程计坐标系，通常为 lidar_odom）
  current_velocity_ = msg->twist.twist;

  // 用 TF 查询 odom_frame_id_ → map_frame_id_ 的旋转，将速度转换到 map 系。
  // 只需要旋转分量（不需要平移），因此取 transform.rotation 对应的 yaw。
  geometry_msgs::msg::Twist vel_map;
  try {
    const auto tf = tf_buffer_->lookupTransform(
      map_frame_id_, odom_frame_id_, tf2::TimePointZero);

    tf2::Quaternion q;
    tf2::fromMsg(tf.transform.rotation, q);
    const double yaw     = tf2::getYaw(q);
    const double cos_yaw = std::cos(yaw);
    const double sin_yaw = std::sin(yaw);

    const double vx = msg->twist.twist.linear.x;
    const double vy = msg->twist.twist.linear.y;

    vel_map.linear.x = cos_yaw * vx - sin_yaw * vy;
    vel_map.linear.y = sin_yaw * vx + cos_yaw * vy;
    vel_map.linear.z = 0.0;
    vel_map.angular  = msg->twist.twist.angular;
  } catch (const tf2::TransformException & e) {
    // TF 尚未就绪时保留上一帧转换结果，避免清零速度
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
      "Cannot get TF %s->%s: %s. Using last known map velocity.",
      odom_frame_id_.c_str(), map_frame_id_.c_str(), e.what());
    return;
  }

  current_velocity_map_ = vel_map;
}

geometry_msgs::msg::PoseStamped ControllerServer::getCurrentPose(const std::string & frame_id)
{
  if (!tf_buffer_) {
    throw tf2::TransformException("TF buffer is not initialized");
  }

  geometry_msgs::msg::TransformStamped transform;
  transform = tf_buffer_->lookupTransform(frame_id, base_frame_id_, tf2::TimePointZero);

  geometry_msgs::msg::PoseStamped pose;
  pose.header.stamp = transform.header.stamp;
  pose.header.frame_id = frame_id;
  pose.pose.position.x = transform.transform.translation.x;
  pose.pose.position.y = transform.transform.translation.y;
  pose.pose.position.z = transform.transform.translation.z;
  pose.pose.orientation = transform.transform.rotation;
  return pose;
}

void ControllerServer::controlLoop()
{
  nav_msgs::msg::Path path_copy;
  {
    std::lock_guard<std::mutex> lock(path_mutex_);
    path_copy = latest_path_;
  }

  if (path_copy.poses.empty()) {
    return;
  }

  RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "path_copy succeeded.");

  if (!controller_) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
      "Controller plugin not yet available.");
    return;
  }

  geometry_msgs::msg::PoseStamped current_pose;
  try {
    current_pose = getCurrentPose(path_copy.header.frame_id);
  } catch (const tf2::TransformException & e) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
      "Failed to get current pose: %s", e.what());
    return;
  }

  geometry_msgs::msg::Twist current_twist;
  {
    std::lock_guard<std::mutex> lock(velocity_mutex_);
    current_twist = current_velocity_map_;  // map 系速度
  }

  geometry_msgs::msg::TwistStamped cmd;
  try {
    cmd = controller_->computeVelocityCommands(current_pose, current_twist);
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
      "Computed velocity commands: linear.x=%.2f, linear.y=%.2f, angular.z=%.2f",
      cmd.twist.linear.x, cmd.twist.linear.y, cmd.twist.angular.z);
  } catch (const std::exception & e) {
    RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
      "Controller plugin threw exception: %s", e.what());
    return;
  }

  if (!control_enabled_.load()) {
    cmd.twist = geometry_msgs::msg::Twist{};
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
      "Control disabled, publishing zero velocity.");
  }

  if (cmd.header.frame_id.empty()) {
    cmd.header.frame_id = base_frame_id_;
  }
  cmd.header.stamp = this->now();

  cmd_vel_pub_->publish(cmd.twist);
}

void ControllerServer::onSetControlEnable(
  const std_srvs::srv::SetBool::Request::SharedPtr request,
  std_srvs::srv::SetBool::Response::SharedPtr response)
{
  control_enabled_.store(request->data);
  response->success = true;
  response->message = request->data ? "Control enabled" : "Control disabled";
  RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
}

}  // namespace sp_controller_server
