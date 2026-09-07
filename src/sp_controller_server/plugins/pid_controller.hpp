#ifndef PID_CONTROLLER_HPP_
#define PID_CONTROLLER_HPP_

#include <memory>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>

#include "sp_controller_server/controller_plugin.hpp"

namespace pid_controller {

class PidController : public sp_controller_server::ControllerPlugin {
public:
  PidController() = default;
  ~PidController() override = default;

  void configure(
      const rclcpp::Node::SharedPtr & node, const std::string & plugin_name,
      const std::shared_ptr<tf2_ros::Buffer> & tf_buffer) override;

  void setPlan(const nav_msgs::msg::Path & path) override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity) override;

  void setSpeedLimit(double speed_limit) override;

private:
  // 存储插件名称
  std::string plugin_name_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  rclcpp::Node::SharedPtr node_;
  // 存储 setPlan 提供的全局路径
  nav_msgs::msg::Path global_plan_;
  // 参数：最大线速度角速度
  double max_angular_speed_;
  double max_linear_speed_;
  
  // PID 参数
  double kp_;
  double ki_;
  double kd_;
  double lookahead_dist_;
  double distance_deadband_;
  // 终点减速参数
  double goal_slowdown_dist_; 
  double goal_stop_dist_;
  bool in_goal_mode_ = false;
  // PID 状态
  double prev_error_;
  double integral_;
  // 时间间隔
  rclcpp::Time last_time_;
  bool has_last_time_ = false;
  std::string base_frame_id_ = "base_link";
  
  // 获取预瞄点
  geometry_msgs::msg::PoseStamped
  getLookAheadPoint(const geometry_msgs::msg::PoseStamped &current_pose,
  const nav_msgs::msg::Path &path);
  double remainingPathLength(size_t from_index) const;

  // 计算线速度和角速度
  void
  computeControlCommands(const geometry_msgs::msg::PoseStamped &current_pose,
  const geometry_msgs::msg::PoseStamped &target_pose,double dt,
  double &vx, double &vy, double &omega);
};

} // namespace pid_controller

#endif // PID_CONTROLLER_HPP_