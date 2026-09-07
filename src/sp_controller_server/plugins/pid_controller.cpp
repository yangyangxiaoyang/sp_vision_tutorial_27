#include "pid_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include <pluginlib/class_list_macros.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace pid_controller {

void PidController::configure(
  const rclcpp::Node::SharedPtr & node, const std::string & name,
  const std::shared_ptr<tf2_ros::Buffer> & tf_buffer)
{
  node_ = node;
  if (!node_) {
    throw std::runtime_error("PidController received null node");
  }

  plugin_name_ = name;
  tf_buffer_ = tf_buffer;

  node_->declare_parameter(plugin_name_ + ".max_linear_speed", 0.5);
  node_->declare_parameter(plugin_name_ + ".max_angular_speed", 1.0);
  node_->declare_parameter(plugin_name_ + ".lookahead_dist", 0.6);
  node_->declare_parameter(plugin_name_ + ".kp", 1.0);
  node_->declare_parameter(plugin_name_ + ".ki", 1.0);
  node_->declare_parameter(plugin_name_ + ".kd", 1.0);
  node_->declare_parameter(plugin_name_ + ".distance_deadband", 0.02);
  node_->declare_parameter(plugin_name_ + ".goal_slowdown_dist", 0.5);
  node_->declare_parameter(plugin_name_ + ".goal_stop_dist", 0.05);
  node_->declare_parameter(plugin_name_ + ".base_frame_id", base_frame_id_);

  node_->get_parameter(plugin_name_ + ".max_linear_speed", max_linear_speed_);
  node_->get_parameter(plugin_name_ + ".max_angular_speed", max_angular_speed_);
  node_->get_parameter(plugin_name_ + ".lookahead_dist", lookahead_dist_);
  node_->get_parameter(plugin_name_ + ".kp", kp_);
  node_->get_parameter(plugin_name_ + ".ki", ki_);
  node_->get_parameter(plugin_name_ + ".kd", kd_);
  node_->get_parameter(plugin_name_ + ".distance_deadband", distance_deadband_);
  node_->get_parameter(plugin_name_ + ".goal_slowdown_dist", goal_slowdown_dist_);
  node_->get_parameter(plugin_name_ + ".goal_stop_dist", goal_stop_dist_);
  node_->get_parameter(plugin_name_ + ".base_frame_id", base_frame_id_);

  prev_error_ = 0.0;
  integral_ = 0.0;
  has_last_time_ = false;
  in_goal_mode_ = false;
}

void PidController::setPlan(const nav_msgs::msg::Path & path)
{
  global_plan_ = path;
}

void PidController::setSpeedLimit(double speed_limit)
{
  max_linear_speed_ = speed_limit;
}

geometry_msgs::msg::TwistStamped
PidController::computeVelocityCommands(const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & /*velocity*/)
{
  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header.stamp = node_->now();
  cmd_vel.header.frame_id = base_frame_id_;

  if (global_plan_.poses.empty()) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
      "[%s] Global plan is empty, publishing zero velocity", plugin_name_.c_str());
    return cmd_vel;
  }

  geometry_msgs::msg::PoseStamped target_pose = getLookAheadPoint(pose, global_plan_);

  rclcpp::Time now = node_->now();
  double dt = 0.0;
  if (has_last_time_) {
    dt = (now - last_time_).seconds();
  }
  last_time_ = now;
  has_last_time_ = true;

  double vx = 0.0;
  double vy = 0.0;
  double omega = 0.0;
  computeControlCommands(pose, target_pose, dt, vx, vy, omega);

  if (!tf_buffer_) {
    throw std::runtime_error("PidController TF buffer not available");
  }

  const std::string src_frame = pose.header.frame_id;
  geometry_msgs::msg::TransformStamped tf_bl_from_src =
    tf_buffer_->lookupTransform(base_frame_id_, src_frame, tf2::TimePointZero);
  tf2::Quaternion q;
  tf2::fromMsg(tf_bl_from_src.transform.rotation, q);
  tf2::Matrix3x3 R(q);
  double vx_bl = R[0][0] * vx + R[0][1] * vy;
  double vy_bl = R[1][0] * vx + R[1][1] * vy;
  cmd_vel.twist.linear.x = vx_bl;
  cmd_vel.twist.linear.y = vy_bl;
  cmd_vel.twist.angular.z = omega;
  // cmd_vel.twist.linear.x = vx;
  // cmd_vel.twist.linear.y = vy;
  // cmd_vel.twist.angular.z = omega;

  return cmd_vel;
}

geometry_msgs::msg::PoseStamped
PidController::getLookAheadPoint(const geometry_msgs::msg::PoseStamped &current_pose,
    const nav_msgs::msg::Path &path) {
    double min_dist = std::numeric_limits<double>::max();
    size_t nearest_index = 0;
    
    for (size_t i = 0; i < path.poses.size(); ++i) {
        double dist = std::hypot(path.poses[i].pose.position.x - current_pose.pose.position.x,
        path.poses[i].pose.position.y - current_pose.pose.position.y);
        if (dist < min_dist) {
            min_dist = dist;
            nearest_index = i;
        }
    }
    
    // 剩余路径长度
    double remaining = remainingPathLength(nearest_index);
    if (remaining <= goal_stop_dist_) {
      in_goal_mode_ = true;
      return path.poses.back();
    } 
    else if (remaining <= goal_slowdown_dist_) {
      in_goal_mode_ = true;
    } 
    else {
      in_goal_mode_ = false;
    }

    for (size_t i = nearest_index; i < path.poses.size(); ++i) {
      double dist = std::hypot(path.poses[i].pose.position.x - current_pose.pose.position.x,
      path.poses[i].pose.position.y - current_pose.pose.position.y);
      if (dist >= lookahead_dist_) {
        return path.poses[i];
      }
    }
    return path.poses.back();
}

double PidController::remainingPathLength(size_t from_index) const {
  if (global_plan_.poses.empty() || from_index >= global_plan_.poses.size()-1) return 0.0;
  double total = 0.0;
  for (size_t i = from_index; i + 1 < global_plan_.poses.size(); ++i) {
    const auto &a = global_plan_.poses[i].pose.position;
    const auto &b = global_plan_.poses[i+1].pose.position;
    total += std::hypot(b.x - a.x, b.y - a.y);
  }
  return total;
}

void PidController::computeControlCommands(const geometry_msgs::msg::PoseStamped &current_pose,
  const geometry_msgs::msg::PoseStamped &target_pose,double dt,
  double &vx, double &vy, double &omega) {
  // 速度方向：沿当前点指向预瞄点
  double dx = target_pose.pose.position.x - current_pose.pose.position.x;
  double dy = target_pose.pose.position.y - current_pose.pose.position.y;
  double dist = std::hypot(dx, dy);
  // double remaining = remainingPathLength(nearest_index);
  // if (dist < std::max(1e-6, distance_deadband_)) { vx = 0.0; vy = 0.0; omega = 0.0; return; }
  if (dist < goal_stop_dist_) {
    prev_error_ = 0.0;
    integral_ = 0.0;
    has_last_time_ = false;
    in_goal_mode_ = false;
    vx = 0.0; vy = 0.0; omega = 0.0;
    return;
  }

  // 对距离做 PID
  if (dt > 0.0) {
    integral_ += dist * dt;
  }
  const double i_lim = 10.0;
  if (integral_ > i_lim) integral_ = i_lim;
  if (integral_ < -i_lim) integral_ = -i_lim;
  double d_err = 0.0;
  if (dt > 0.0) {
    d_err = (dist - prev_error_) / dt;
  }
  double speed_cmd = kp_ * dist + ki_ * integral_ + kd_ * d_err;
  // 终点减速模式
  if (in_goal_mode_) {
    if (dist <= goal_stop_dist_) {
      speed_cmd = 0.0;
    } 
    else if (dist <= goal_slowdown_dist_) {
      double scale = (dist - goal_stop_dist_) / (goal_slowdown_dist_ - goal_stop_dist_);
      speed_cmd *= std::clamp(scale, 0.0, 1.0);
    }
  }

  prev_error_ = dist;

  // if (speed_cmd < 0.0) speed_cmd = 0.0; 
  if (speed_cmd > max_linear_speed_) speed_cmd = max_linear_speed_;

  // 方向速度
  double ux = dx / dist;
  double uy = dy / dist;
  vx = speed_cmd * ux;
  vy = speed_cmd * uy;
  omega = 0.0;
  RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
    "[%s][detail] dist=%.3f speed_cmd=%.3f ux=%.2f uy=%.2f vx=%.3f vy=%.3f dt=%.3f integral_=%.3f d_err=%.3f lookahead_dist_=%.3f goal_mode=%d slow_dist=%.2f stop_dist=%.2f",
    plugin_name_.c_str(), dist, speed_cmd, ux, uy, vx, vy, dt, integral_, d_err, lookahead_dist_, (int)in_goal_mode_, goal_slowdown_dist_, goal_stop_dist_);
}

} // namespace pid_controller

PLUGINLIB_EXPORT_CLASS(pid_controller::PidController, sp_controller_server::ControllerPlugin)
