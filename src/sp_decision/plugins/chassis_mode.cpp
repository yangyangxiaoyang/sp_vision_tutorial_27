#include "chassis_mode.hpp"

#include "behaviortree_cpp_v3/bt_factory.h"

namespace sp_decision
{

// ─────────────────────────────────────────────────────────────────────────────
// Constructor
// ─────────────────────────────────────────────────────────────────────────────

ChassisMode::ChassisMode(
    const std::string & name,
    const BT::NodeConfiguration & config)
    : BT::SyncActionNode(name, config)
{
    node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");

    pub_ = node_->create_publisher<robot_msg::msg::ChassisModeMsg>(
        "/chassis/mode",
        rclcpp::QoS(1).reliable().transient_local());

    RCLCPP_INFO(node_->get_logger(), "[ChassisMode] BT node initialised.");
}

// ─────────────────────────────────────────────────────────────────────────────
// Static port declaration
// ─────────────────────────────────────────────────────────────────────────────

BT::PortsList ChassisMode::providedPorts()
{
    return {
        BT::InputPort<int>  ("mode",            0,     "运动模式: 0=小陀螺, 1=底盘跟随, 2=变速小陀螺, 3=差速对正, 4=特殊对正, 5=无线充电"),
        BT::InputPort<bool> ("is_stop",         false, "是否停止运动 (true=停止)"),
        BT::InputPort<float>("rotate_velocity", 0.0f,  "旋转速度 (rad/s)，mode=0 时生效"),
    };
}

// ─────────────────────────────────────────────────────────────────────────────
// tick – publish and return SUCCESS immediately
// ─────────────────────────────────────────────────────────────────────────────

BT::NodeStatus ChassisMode::tick()
{
    robot_msg::msg::ChassisModeMsg msg;

    int mode_int = 0;
    getInput("mode",            mode_int);
    msg.mode = static_cast<uint8_t>(mode_int);
    getInput("is_stop",         msg.is_stop);
    getInput("rotate_velocity", msg.rotate_velocity);

    pub_->publish(msg);

    RCLCPP_DEBUG(node_->get_logger(),
                 "[ChassisMode] Published mode=%u  is_stop=%d  rotate_velocity=%.2f",
                 msg.mode, msg.is_stop, msg.rotate_velocity);

    return BT::NodeStatus::SUCCESS;
}

}  // namespace sp_decision

// ─────────────────────────────────────────────────────────────────────────────
// Plugin registration
// ─────────────────────────────────────────────────────────────────────────────

extern "C" __attribute__((visibility("default"))) void BT_RegisterNodesFromPlugin(BT::BehaviorTreeFactory & factory)
{
    factory.registerNodeType<sp_decision::ChassisMode>("ChassisMode");
}
