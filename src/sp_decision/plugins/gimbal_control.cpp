#include "gimbal_control.hpp"

#include "behaviortree_cpp_v3/bt_factory.h"

namespace sp_decision
{

// ─────────────────────────────────────────────────────────────────────────────
// Constructor
// ─────────────────────────────────────────────────────────────────────────────

GimbalControl::GimbalControl(
    const std::string & name,
    const BT::NodeConfiguration & config)
    : BT::SyncActionNode(name, config)
{
    node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");

    pub_ = node_->create_publisher<robot_msg::msg::GimbalControlMsg>(
        "/gimbal/control", rclcpp::QoS(1).reliable());

    RCLCPP_INFO(node_->get_logger(), "[GimbalControl] BT node initialised.");
}

// ─────────────────────────────────────────────────────────────────────────────
// Static port declaration
// ─────────────────────────────────────────────────────────────────────────────

BT::PortsList GimbalControl::providedPorts()
{
    return {
        BT::InputPort<int>  ("mode",                    0,     "运动模式: 0=全向扫描 1=三轴指定角度 2=范围扫描 3=仅yaw锁定 4=仅大yaw指定角度"),
        BT::InputPort<float>("big_yaw",                 0.0f,  "big_yaw 目标角度 (deg)，mode=1/4 时生效"),
        BT::InputPort<float>("small_yaw",               0.0f,  "small_yaw 目标角度 (deg)，mode=1 时生效"),
        BT::InputPort<float>("pitch",                   0.0f,  "pitch 目标角度 (deg)，mode=1 时生效"),
        BT::InputPort<float>("small_yaw_lower_limit",   0.0f,  "small_yaw 范围下限 (deg)，mode=2 时生效"),
        BT::InputPort<float>("small_yaw_upper_limit",   0.0f,  "small_yaw 范围上限 (deg)，mode=2 时生效"),
        BT::InputPort<float>("pitch_lower_limit",       0.0f,  "pitch 范围下限 (deg)，mode=2 时生效"),
        BT::InputPort<float>("pitch_upper_limit",       0.0f,  "pitch 范围上限 (deg)，mode=2 时生效"),
    };
}

// ─────────────────────────────────────────────────────────────────────────────
// tick – publish and return SUCCESS immediately
// ─────────────────────────────────────────────────────────────────────────────

BT::NodeStatus GimbalControl::tick()
{
    robot_msg::msg::GimbalControlMsg msg;

    int mode_int = 0;
    getInput("mode",              mode_int);
    msg.mode = static_cast<uint8_t>(mode_int);
    getInput("big_yaw",                 msg.big_yaw);
    getInput("small_yaw",               msg.small_yaw);
    getInput("pitch",                   msg.pitch);
    getInput("small_yaw_lower_limit",   msg.small_yaw_lower_limit);
    getInput("small_yaw_upper_limit",   msg.small_yaw_upper_limit);
    getInput("pitch_lower_limit",       msg.pitch_lower_limit);
    getInput("pitch_upper_limit",       msg.pitch_upper_limit);

    pub_->publish(msg);

    RCLCPP_DEBUG(node_->get_logger(),
                 "[GimbalControl] Published mode=%u  big_yaw=%.1f  pitch=%.1f",
                 msg.mode, msg.big_yaw, msg.pitch);

    return BT::NodeStatus::SUCCESS;
}

}  // namespace sp_decision

// ─────────────────────────────────────────────────────────────────────────────
// Plugin registration
// ─────────────────────────────────────────────────────────────────────────────

extern "C" __attribute__((visibility("default"))) void BT_RegisterNodesFromPlugin(BT::BehaviorTreeFactory & factory)
{
    factory.registerNodeType<sp_decision::GimbalControl>("GimbalControl");
}
