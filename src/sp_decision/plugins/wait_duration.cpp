#include "wait_duration.hpp"

#include "behaviortree_cpp_v3/bt_factory.h"

namespace sp_decision
{

// ─────────────────────────────────────────────────────────────────────────────
// Constructor
// ─────────────────────────────────────────────────────────────────────────────

WaitDuration::WaitDuration(
    const std::string & name,
    const BT::NodeConfiguration & config)
    : BT::StatefulActionNode(name, config)
{
    node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
    RCLCPP_INFO(node_->get_logger(), "[WaitDuration] BT node initialised.");
}

// ─────────────────────────────────────────────────────────────────────────────
// Static port declaration
// ─────────────────────────────────────────────────────────────────────────────

BT::PortsList WaitDuration::providedPorts()
{
    return {
        BT::InputPort<float>("duration", 0.0f,
            "等待时长 (s)。< 0 永远返回 RUNNING；>= 0 等待后返回 SUCCESS"),
    };
}

// ─────────────────────────────────────────────────────────────────────────────
// onStart
// ─────────────────────────────────────────────────────────────────────────────

BT::NodeStatus WaitDuration::onStart()
{
    float duration_s = 0.0f;
    getInput("duration", duration_s);

    if (duration_s < 0.0f)
    {
        infinite_ = true;
        RCLCPP_INFO(node_->get_logger(),
                    "[WaitDuration] duration < 0 – blocking forever (RUNNING).");
    }
    else
    {
        infinite_        = false;
        target_duration_ = std::chrono::duration<double>(static_cast<double>(duration_s));
        start_time_      = std::chrono::steady_clock::now();
        RCLCPP_INFO(node_->get_logger(),
                    "[WaitDuration] Waiting %.3f s.", duration_s);

        // Zero-duration case: return immediately
        if (duration_s == 0.0f)
        {
            return BT::NodeStatus::SUCCESS;
        }
    }

    return BT::NodeStatus::RUNNING;
}

// ─────────────────────────────────────────────────────────────────────────────
// onRunning
// ─────────────────────────────────────────────────────────────────────────────

BT::NodeStatus WaitDuration::onRunning()
{
    if (infinite_)
    {
        return BT::NodeStatus::RUNNING;
    }

    const auto elapsed = std::chrono::steady_clock::now() - start_time_;
    if (elapsed >= target_duration_)
    {
        RCLCPP_INFO(node_->get_logger(), "[WaitDuration] Wait complete – SUCCESS.");
        return BT::NodeStatus::SUCCESS;
    }

    return BT::NodeStatus::RUNNING;
}

// ─────────────────────────────────────────────────────────────────────────────
// onHalted
// ─────────────────────────────────────────────────────────────────────────────

void WaitDuration::onHalted()
{
    infinite_        = false;
    target_duration_ = std::chrono::duration<double>(0);
    start_time_      = std::chrono::steady_clock::time_point{};
    RCLCPP_INFO(node_->get_logger(), "[WaitDuration] Halted – timer cleared.");
}

}  // namespace sp_decision

// ─────────────────────────────────────────────────────────────────────────────
// Plugin registration
// ─────────────────────────────────────────────────────────────────────────────

extern "C" __attribute__((visibility("default"))) void BT_RegisterNodesFromPlugin(BT::BehaviorTreeFactory & factory)
{
    factory.registerNodeType<sp_decision::WaitDuration>("WaitDuration");
}
