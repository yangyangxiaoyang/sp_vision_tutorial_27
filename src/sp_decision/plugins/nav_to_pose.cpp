#include "nav_to_pose.hpp"

#include <chrono>
#include <regex>
#include <stdexcept>

#include "behaviortree_cpp_v3/bt_factory.h"

namespace sp_decision
{

    // ─────────────────────────────────────────────────────────────────────────────
    // Constructor
    // ─────────────────────────────────────────────────────────────────────────────

    NavToPose::NavToPose(
        const std::string &name,
        const BT::NodeConfiguration &config)
        : BT::StatefulActionNode(name, config)
    {
        // Use a process-wide counter to generate a unique ROS2 node name.
        static std::atomic<uint32_t> instance_counter{0};
        const std::string ros_node_name =
            "nav_to_pose_" + std::to_string(instance_counter.fetch_add(1));

        // Each NavToPose instance owns a fully private ROS2 node.
        // This avoids all shared-node concurrency issues: the main
        // MultiThreadedExecutor in decision_main.cpp never touches this node,
        // and executor_ is the sole owner of every callback on it.
        node_ = std::make_shared<rclcpp::Node>(ros_node_name);
        executor_.add_node(node_);

        action_client_ = rclcpp_action::create_client<NavigateToPose>(
            node_,
            "navigate_to_pose");

        RCLCPP_INFO(node_->get_logger(),
                    "[NavToPose] BT node '%s' initialised (ros node: %s).",
                    name.c_str(), ros_node_name.c_str());
    }

    // ─────────────────────────────────────────────────────────────────────────────
    // Static port declaration
    // ─────────────────────────────────────────────────────────────────────────────

    BT::PortsList NavToPose::providedPorts()
    {
        return {
            BT::InputPort<std::string>(
                "goal_pose",
                "2-D coordinate string: (x,y), e.g. (10.0,9.0)"),
            BT::InputPort<geometry_msgs::msg::PoseStamped>(
                "goal_pose_stamped",
                "PoseStamped read directly from the blackboard, e.g. goal_pose_stamped=\"{bb_key}\""),
        };
    }

    // ─────────────────────────────────────────────────────────────────────────────
    // onStart – send goal once
    // ─────────────────────────────────────────────────────────────────────────────

    BT::NodeStatus NavToPose::onStart()
    {
        // ── Resolve goal pose: blackboard PoseStamped takes priority ────────
        geometry_msgs::msg::PoseStamped goal_pose;

        geometry_msgs::msg::PoseStamped stamped_from_bb;
        if (getInput<geometry_msgs::msg::PoseStamped>("goal_pose_stamped", stamped_from_bb))
        {
            // Mode 1: PoseStamped retrieved directly from the blackboard
            goal_pose = stamped_from_bb;
            RCLCPP_INFO(node_->get_logger(),
                        "[NavToPose] Using blackboard PoseStamped  x=%.3f  y=%.3f  frame='%s'",
                        goal_pose.pose.position.x,
                        goal_pose.pose.position.y,
                        goal_pose.header.frame_id.c_str());
        }
        else
        {
            // Mode 2: Parse coordinate string  "(x,y)"
            std::string goal_str;
            if (!getInput<std::string>("goal_pose", goal_str))
            {
                RCLCPP_ERROR(node_->get_logger(),
                             "[NavToPose] Neither 'goal_pose_stamped' nor 'goal_pose' "
                             "was provided on the input ports.");
                return BT::NodeStatus::FAILURE;
            }

            auto maybe_pose = parse_goal_string(goal_str);
            if (!maybe_pose)
            {
                RCLCPP_ERROR(node_->get_logger(),
                             "[NavToPose] Failed to parse goal_pose string: '%s' "
                             "(expected format: \"(x,y)\")",
                             goal_str.c_str());
                return BT::NodeStatus::FAILURE;
            }
            goal_pose = *maybe_pose;
        }

        // Wait briefly for the action server to become available
        if (!action_client_->wait_for_action_server(std::chrono::seconds(5)))
        {
            RCLCPP_ERROR(node_->get_logger(),
                         "[NavToPose] Action server 'navigate_to_pose' not available");
            return BT::NodeStatus::FAILURE;
        }

        // Reset state; bump generation so any in-flight callbacks from the
        // previous (halted) goal are ignored when they eventually fire.
        goal_handle_.reset();
        const uint64_t my_gen = goal_gen_.fetch_add(1) + 1;
        action_state_.store(static_cast<int>(ActionState::PENDING));

        // Ensure a valid frame_id before sending
        if (goal_pose.header.frame_id.empty())
            goal_pose.header.frame_id = "map";

        RCLCPP_INFO(node_->get_logger(),
                    "[NavToPose] Sending goal  x=%.3f  y=%.3f  frame='%s'",
                    goal_pose.pose.position.x,
                    goal_pose.pose.position.y,
                    goal_pose.header.frame_id.c_str());

        // Build goal message
        auto goal_msg = NavigateToPose::Goal();
        goal_msg.pose = goal_pose;
        goal_msg.behavior_tree = ""; // use the default behaviour tree

        // Build send-goal options referencing our three callbacks
        auto send_goal_options =
            rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

        send_goal_options.goal_response_callback =
            [this, my_gen](const GoalHandle::SharedPtr &goal_handle)
        {
            if (goal_gen_.load() != my_gen) return;  // stale – belongs to old goal
            goal_response_cb(goal_handle);
        };

        send_goal_options.feedback_callback =
            [this, my_gen](GoalHandle::SharedPtr handle,
                   const std::shared_ptr<const NavigateToPose::Feedback> feedback)
        {
            if (goal_gen_.load() != my_gen) return;
            feedback_cb(handle, feedback);
        };

        send_goal_options.result_callback =
            [this, my_gen](const GoalHandle::WrappedResult &result)
        {
            if (goal_gen_.load() != my_gen) return;  // stale cancel result – discard
            result_cb(result);
        };

        action_client_->async_send_goal(goal_msg, send_goal_options);

        // 导航开始 → nav_success = 0
        config().blackboard->set<int>("nav_success", 0);

        return BT::NodeStatus::RUNNING;
    }

    // ─────────────────────────────────────────────────────────────────────────────
    // onRunning – poll until the action finishes
    // ─────────────────────────────────────────────────────────────────────────────

    BT::NodeStatus NavToPose::onRunning()
    {
        // Process any pending action callbacks (non-blocking)
        executor_.spin_some(std::chrono::milliseconds(10));

        const auto state = static_cast<ActionState>(action_state_.load());

        switch (state)
        {
        case ActionState::PENDING:
        case ActionState::RUNNING:
            // RCLCPP_INFO(node_->get_logger(),
            //             "[NavToPose] Goal is %s...",
            //             (state == ActionState::PENDING) ? "pending" : "running");
            return BT::NodeStatus::RUNNING;

        case ActionState::SUCCEEDED:
            RCLCPP_INFO(node_->get_logger(), "[NavToPose] Goal succeeded.");
            config().blackboard->set<int>("nav_success", 1);
            return BT::NodeStatus::SUCCESS;

        case ActionState::FAILED:
            RCLCPP_ERROR(node_->get_logger(), "[NavToPose] Goal failed / aborted.");
            config().blackboard->set<int>("nav_success", 1);
            return BT::NodeStatus::FAILURE;

        case ActionState::CANCELED:
            RCLCPP_INFO(node_->get_logger(), "[NavToPose] Goal was canceled.");
            config().blackboard->set<int>("nav_success", 1);
            return BT::NodeStatus::FAILURE;

        default:
            return BT::NodeStatus::FAILURE;
        }
    }

    // ─────────────────────────────────────────────────────────────────────────────
    // onHalted – cancel the active goal when the BT interrupts this node
    // ─────────────────────────────────────────────────────────────────────────────

    void NavToPose::onHalted()
    {
        // Bump generation first so the pending cancel-result callback (which
        // arrives asynchronously) will be silently dropped when it fires.
        goal_gen_.fetch_add(1);

        const auto state = static_cast<ActionState>(action_state_.load());

        if (state == ActionState::PENDING || state == ActionState::RUNNING)
        {
            if (goal_handle_)
            {
                RCLCPP_INFO(node_->get_logger(), "[NavToPose] Halted – canceling active goal.");
                action_client_->async_cancel_goal(goal_handle_);
            }
        }

        action_state_.store(static_cast<int>(ActionState::IDLE));
        goal_handle_.reset();
    }

    // ─────────────────────────────────────────────────────────────────────────────
    // Private callbacks
    // ─────────────────────────────────────────────────────────────────────────────

    void NavToPose::goal_response_cb(
        const GoalHandle::SharedPtr &handle)
    {
        if (!handle)
        {
            RCLCPP_ERROR(node_->get_logger(),
                         "[NavToPose] Goal was REJECTED by the action server.");
            action_state_.store(static_cast<int>(ActionState::FAILED));
        }
        else
        {
            RCLCPP_INFO(node_->get_logger(),
                        "[NavToPose] Goal ACCEPTED – robot is navigating.");
            goal_handle_ = handle;
            action_state_.store(static_cast<int>(ActionState::RUNNING));
        }
    }

    void NavToPose::feedback_cb(
        GoalHandle::SharedPtr /*handle*/,
        const std::shared_ptr<const NavigateToPose::Feedback> feedback)
    {
        RCLCPP_DEBUG(node_->get_logger(),
                     "[NavToPose] Feedback – distance_remaining=%.3f  eta=%.1f s",
                     feedback->distance_remaining,
                     feedback->estimated_time_remaining);
    }

    void NavToPose::result_cb(const GoalHandle::WrappedResult &result)
    {
        switch (result.code)
        {
        case rclcpp_action::ResultCode::SUCCEEDED:
            action_state_.store(static_cast<int>(ActionState::SUCCEEDED));
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(node_->get_logger(), "[NavToPose] Action ABORTED.");
            action_state_.store(static_cast<int>(ActionState::FAILED));
            break;
        case rclcpp_action::ResultCode::CANCELED:
            action_state_.store(static_cast<int>(ActionState::CANCELED));
            break;
        default:
            RCLCPP_ERROR(node_->get_logger(),
                         "[NavToPose] Unknown result code: %d",
                         static_cast<int>(result.code));
            action_state_.store(static_cast<int>(ActionState::FAILED));
            break;
        }
        goal_handle_.reset();
    }

    // ─────────────────────────────────────────────────────────────────────────────
    // parse_goal_string
    // ─────────────────────────────────────────────────────────────────────────────

    std::optional<geometry_msgs::msg::PoseStamped>
    NavToPose::parse_goal_string(const std::string &s)
    {
        // Accept:  (x,y)  with optional whitespace
        static const std::regex re(
            R"delim(\s*\(\s*([+-]?[\d.]+)\s*,\s*([+-]?[\d.]+)\s*\)\s*)delim");

        std::smatch m;
        if (!std::regex_match(s, m, re))
        {
            return std::nullopt;
        }

        double x = std::stod(m[1].str());
        double y = std::stod(m[2].str());

        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = "map";
        pose.pose.position.x = x;
        pose.pose.position.y = y;
        pose.pose.position.z = 0.0;
        pose.pose.orientation.w = 1.0;
        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = 0.0;

        return pose;
    }

} // namespace sp_decision

// ─────────────────────────────────────────────────────────────────────────────
// Plugin registration – explicit extern "C" guarantees symbol visibility
// ─────────────────────────────────────────────────────────────────────────────

extern "C" __attribute__((visibility("default"))) void BT_RegisterNodesFromPlugin(BT::BehaviorTreeFactory &factory)
{
    factory.registerNodeType<sp_decision::NavToPose>("NavToPose");
}
