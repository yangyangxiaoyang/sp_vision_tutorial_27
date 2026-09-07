#ifndef SP_DECISION_PLUGIN_GIMBAL_CONTROL_HPP_
#define SP_DECISION_PLUGIN_GIMBAL_CONTROL_HPP_

#include <memory>
#include <string>

#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "robot_msg/msg/gimbal_control_msg.hpp"

namespace sp_decision
{

/**
 * @class GimbalControl
 * @brief BehaviorTree SyncActionNode that publishes a GimbalControlMsg every
 *        tick and immediately returns SUCCESS.
 *
 * Input Ports
 * ───────────
 *   mode                  (int)     运动模式: 0=全向扫描 1=三轴指定角度 2=范围扫描 3=仅yaw锁定 4=仅大yaw指定角度
 *   big_yaw               (float)   big_yaw 目标角度 (deg)，mode=1/4 时生效
 *   small_yaw             (float)   small_yaw 目标角度 (deg)，mode=1 时生效
 *   pitch                 (float)   pitch 目标角度 (deg)，mode=1 时生效
 *   small_yaw_lower_limit (float)   small_yaw 范围下限 (deg)，mode=2 时生效
 *   small_yaw_upper_limit (float)   small_yaw 范围上限 (deg)，mode=2 时生效
 *   pitch_lower_limit     (float)   pitch 范围下限 (deg)，mode=2 时生效
 *   pitch_upper_limit     (float)   pitch 范围上限 (deg)，mode=2 时生效
 *
 * Publish topic
 * ─────────────
 *   /gimbal/control  (robot_msg/msg/GimbalControlMsg)
 *
 * XML example
 * ───────────
 *   <!-- 全向扫描 -->
 *   <GimbalControl mode="0"/>
 *
 *   <!-- 三轴指定角度 -->
 *   <GimbalControl mode="1" big_yaw="45.0" small_yaw="0.0" pitch="-10.0"/>
 *
 *   <!-- 仅大 yaw 指定角度 -->
 *   <GimbalControl mode="4" big_yaw="45.0"/>
 *
 *   <!-- 范围扫描 -->
 *   <GimbalControl mode="2"
 *                  small_yaw_lower_limit="-30.0" small_yaw_upper_limit="30.0"
 *                  pitch_lower_limit="-15.0" pitch_upper_limit="15.0"/>
 */
class GimbalControl : public BT::SyncActionNode
{
public:
  GimbalControl(const std::string & name, const BT::NodeConfiguration & config);
  ~GimbalControl() override = default;

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<robot_msg::msg::GimbalControlMsg>::SharedPtr pub_;
};

}  // namespace sp_decision

#endif  // SP_DECISION_PLUGIN_GIMBAL_CONTROL_HPP_
