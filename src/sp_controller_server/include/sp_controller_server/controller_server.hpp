#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <string>

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "sp_controller_server/controller_plugin.hpp"

namespace sp_controller_server {

class ControllerServer : public rclcpp::Node
{
public:
  ControllerServer();
  ~ControllerServer() override = default;

  void init();

private:
  void loadPlugin();
  void onLocalPath(const nav_msgs::msg::Path::SharedPtr msg);
  void onOdometry(const nav_msgs::msg::Odometry::SharedPtr msg);
  void controlLoop();
  geometry_msgs::msg::PoseStamped getCurrentPose(const std::string & frame_id);
  void onSetControlEnable(
    const std_srvs::srv::SetBool::Request::SharedPtr request,
    std_srvs::srv::SetBool::Response::SharedPtr response);

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr local_path_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr set_control_enable_srv_;

  pluginlib::ClassLoader<sp_controller_server::ControllerPlugin> loader_;
  std::shared_ptr<sp_controller_server::ControllerPlugin> controller_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  nav_msgs::msg::Path latest_path_;
  geometry_msgs::msg::Twist current_velocity_;       ///< 原始里程计速度（里程计坐标系）
  geometry_msgs::msg::Twist current_velocity_map_;   ///< 转换到 map 系后的速度

  std::mutex path_mutex_;
  std::mutex velocity_mutex_;

  std::string local_path_topic_;
  std::string odom_topic_;
  std::string odom_frame_id_;   ///< 里程计速度所在坐标系，默认 "lidar_odom"
  std::string map_frame_id_;    ///< 目标 map 坐标系，默认 "map"
  std::string cmd_vel_topic_;
  std::string plugin_name_;
  std::string plugin_type_;
  std::string base_frame_id_;

  double control_frequency_{20.0};
  double speed_limit_{0.5};

  std::atomic<bool> control_enabled_{true};
};

}  // namespace sp_controller_server
