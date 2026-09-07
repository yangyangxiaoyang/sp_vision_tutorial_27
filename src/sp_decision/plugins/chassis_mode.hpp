#ifndef SP_DECISION_PLUGIN_CHASSIS_MODE_HPP_
#define SP_DECISION_PLUGIN_CHASSIS_MODE_HPP_

#include <memory>
#include <string>

#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "robot_msg/msg/chassis_mode_msg.hpp"

namespace sp_decision
{

/**
 * @class ChassisMode
 * @brief BehaviorTree SyncActionNode that publishes a ChassisModeMsg every
 *        tick and immediately returns SUCCESS.
 *
 * Input Ports
 * ───────────
 *   mode             (int)    运动模式: 0=小陀螺, 1=底盘跟随, 2=变速小陀螺, 3=差速对正, 4=特殊对正, 5=无线充电
 *   is_stop          (bool)   是否停止运动 (true=停止)
 *   rotate_velocity  (float)  旋转速度 (rad/s)，mode=0 时生效
 *
 * Publish topic
 * ─────────────
 *   /chassis/mode  (robot_msg/msg/ChassisModeMsg)
 *
 * XML example
 * ───────────
 *   <!-- 小陀螺 -->
 *   <ChassisMode mode="0" is_stop="false" rotate_velocity="3.0"/>
 *
 *   <!-- 底盘跟随 -->
 *   <ChassisMode mode="1" is_stop="false"/>
 *
 *   <!-- 停止 -->
 *   <ChassisMode mode="0" is_stop="true" rotate_velocity="0.0"/>
 */
class ChassisMode : public BT::SyncActionNode
{
public:
  ChassisMode(const std::string & name, const BT::NodeConfiguration & config);
  ~ChassisMode() override = default;

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<robot_msg::msg::ChassisModeMsg>::SharedPtr pub_;
};

}  // namespace sp_decision

#endif  // SP_DECISION_PLUGIN_CHASSIS_MODE_HPP_
