#ifndef SP_NAV_BT_NAV_INTERFACE_NODE_HPP_
#define SP_NAV_BT_NAV_INTERFACE_NODE_HPP_

#include <memory>
#include <string>
#include <vector>
#include <mutex>
#include <atomic>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robot_msg/action/navigate_to_pose.hpp"
#include "sp_nav_bt/nav_bt_engine.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

namespace sp_nav_bt
{

class NavInterfaceNode : public rclcpp::Node
{
public:
  using NavigateToPose = robot_msg::action::NavigateToPose;
  using GoalHandleNav = rclcpp_action::ServerGoalHandle<NavigateToPose>;

  explicit NavInterfaceNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  virtual ~NavInterfaceNode() = default;

  void init_engine();

protected:
  // TF2
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Action Server Callbacks
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const NavigateToPose::Goal> goal);

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleNav> goal_handle);

  void handle_accepted(const std::shared_ptr<GoalHandleNav> goal_handle);

  // Core execution logic
  void execute_tree();

  /// 重定位完成后由外部服务调用：halt 当前行为树并以相同目标重新启动。
  /// 服务名: /nav_interface/restart_task (std_srvs/Trigger)
  void handle_restart_task(
    const std_srvs::srv::Trigger::Request::SharedPtr  req,
    const std_srvs::srv::Trigger::Response::SharedPtr res);

  // BT Engine
  std::shared_ptr<NavBTEngine> bt_engine_;
  
  // Action Servers
  rclcpp_action::Server<NavigateToPose>::SharedPtr nav_to_pose_server_;

  // Restart service server
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr restart_task_srv_;

  // State management
  std::mutex bt_mutex_;
  std::atomic<bool> is_task_running_{false};
  /// 置为 true 期间 execute_tree 完成后不向 action client 发 abort/succeed，
  /// 由 restart 逻辑接管生命周期管理。
  bool is_restarting_{false};
  std::shared_ptr<GoalHandleNav> current_goal_handle_;

  /// 本次导航任务的原始目标（action 受理时缓存）。
  /// 地形节点可能改写黑板 goal_pose 为中间点；restart_task / 每次 execute_tree
  /// 都必须用该缓存恢复，禁止用黑板当前值。
  geometry_msgs::msg::PoseStamped original_goal_pose_;
  bool has_original_goal_pose_{false};
  
  std::string default_bt_xml_;
  std::vector<std::string> plugin_libs_;

  // Threading
  std::thread bt_thread_;
};

} // namespace sp_nav_bt

#endif // SP_NAV_BT_NAV_INTERFACE_NODE_HPP_
