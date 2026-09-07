#ifndef SP_DECISION_PLUGIN_WAIT_DURATION_HPP_
#define SP_DECISION_PLUGIN_WAIT_DURATION_HPP_

#include <chrono>
#include <memory>
#include <string>

#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"

namespace sp_decision
{

/**
 * @class WaitDuration
 * @brief BehaviorTree StatefulActionNode that waits for a given duration.
 *
 * Input Ports
 * ───────────
 *   duration  (float)  等待时长 (s)。
 *                      < 0  : 永远返回 RUNNING（永久阻塞）
 *                      >= 0 : 等待指定秒数后返回 SUCCESS
 *
 * Behaviour
 * ─────────
 *   onStart   : 记录起始时间；若 duration < 0 直接进入永久等待模式
 *   onRunning : 检查是否已超时，超时则返回 SUCCESS，否则 RUNNING
 *   onHalted  : 清空计时器，回到 IDLE 状态
 *
 * XML example
 * ───────────
 *   <!-- 等待 2.5 秒 -->
 *   <WaitDuration duration="2.5"/>
 *
 *   <!-- 永久 RUNNING（直到父节点 halt） -->
 *   <WaitDuration duration="-1"/>
 *
 *   <!-- 从 blackboard 读取时长 -->
 *   <WaitDuration duration="{wait_time}"/>
 */
class WaitDuration : public BT::StatefulActionNode
{
public:
  WaitDuration(const std::string & name, const BT::NodeConfiguration & config);
  ~WaitDuration() override = default;

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart()   override;
  BT::NodeStatus onRunning() override;
  void           onHalted()  override;

private:
  rclcpp::Node::SharedPtr node_;

  bool  infinite_{false};
  std::chrono::steady_clock::time_point start_time_;
  std::chrono::duration<double>         target_duration_{0};
};

}  // namespace sp_decision

#endif  // SP_DECISION_PLUGIN_WAIT_DURATION_HPP_
