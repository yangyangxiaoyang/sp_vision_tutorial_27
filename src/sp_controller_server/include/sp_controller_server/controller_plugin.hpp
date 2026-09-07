#pragma once

#include <memory>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>

namespace sp_controller_server {

class ControllerPlugin
{
public:
  virtual ~ControllerPlugin() = default;

  virtual void configure(
    const rclcpp::Node::SharedPtr & node,
    const std::string & plugin_name,
    const std::shared_ptr<tf2_ros::Buffer> & tf_buffer) = 0;

  virtual void setPlan(const nav_msgs::msg::Path & path) = 0;

  virtual geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity) = 0;

  virtual void setSpeedLimit(double speed_limit) {
    (void)speed_limit;
  }
};

}  // namespace sp_controller_server
