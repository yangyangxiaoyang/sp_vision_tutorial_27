#ifndef SP_NAV_BT_PLUGINS_SET_CONTROL_ENABLE_HPP_
#define SP_NAV_BT_PLUGINS_SET_CONTROL_ENABLE_HPP_

#include <chrono>

#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"

namespace sp_nav_bt
{

/**
 * @brief 调用 /set_control_enable 服务控制 controller_server 的使能状态。
 *
 * InputPorts:
 *   enable       (bool)   - true 启用控制, false 禁用控制
 *   service_name (string) - SetBool 服务名称, 默认 "/set_control_enable"
 *   bb_key       (string) - Blackboard 中记录当前状态的 key, 默认 "controller_enabled"
 *                           同一 blackboard 下多个节点使用相同 key 时自动共享状态,
 *                           避免重复调用服务。
 *
 * Blackboard state:
 *   该 key 在首次服务调用成功后写入。若 key 不存在则视为"未知", 强制调用一次服务
 *   以确保与服务端实际状态同步。
 *   新任务启动时会将缓存置为 false；halt 中断 pending 请求时按已发送的目标值回写缓存。
 */
class SetControlEnable : public BT::CoroActionNode
{
public:
  SetControlEnable(const std::string & name, const BT::NodeConfiguration & conf);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

  void halt() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr client_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future_;
  bool pending_{false};
  bool pending_enable_{false};
  std::string pending_bb_key_;
  std::chrono::steady_clock::time_point request_start_time_;
};

}  // namespace sp_nav_bt

#endif  // SP_NAV_BT_PLUGINS_SET_CONTROL_ENABLE_HPP_