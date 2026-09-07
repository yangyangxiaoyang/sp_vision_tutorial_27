#ifndef SP_DECISION_PLUGIN_NAV_TO_POSE_HPP_
#define SP_DECISION_PLUGIN_NAV_TO_POSE_HPP_

#include <atomic>
#include <cmath>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "behaviortree_cpp_v3/action_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robot_msg/action/navigate_to_pose.hpp"

// ─────────────────────────────────────────────────────────────────────────────
// BT::convertFromString specialization for geometry_msgs::msg::PoseStamped
//
// Required by BehaviorTree.CPP v3 whenever the type is used as an InputPort.
// Accepted string formats (frame_id always set to "map"):
//   "(x,y)"                        – z=0, orientation identity
//   "(x,y,yaw)"                    – yaw in radians, converted to quaternion
//   "(x,y,z,qx,qy,qz,qw)"         – full pose
// ─────────────────────────────────────────────────────────────────────────────
namespace BT
{
template <>
inline geometry_msgs::msg::PoseStamped
convertFromString<geometry_msgs::msg::PoseStamped>(StringView str)
{
    // Tokenise on commas after stripping surrounding parentheses/whitespace
    auto stripped = str;
    while (!stripped.empty() &&
           (stripped.front() == '(' || stripped.front() == ' '))
        stripped.remove_prefix(1);
    while (!stripped.empty() &&
           (stripped.back() == ')' || stripped.back() == ' '))
        stripped.remove_suffix(1);

    std::vector<double> v;
    std::string token;
    for (char c : stripped)
    {
        if (c == ',') {
            v.push_back(std::stod(token));
            token.clear();
        } else {
            token += c;
        }
    }
    if (!token.empty())
        v.push_back(std::stod(token));

    if (v.size() != 2 && v.size() != 3 && v.size() != 7)
        throw RuntimeError(
            "convertFromString<PoseStamped>: expected 2, 3, or 7 values, got " +
            std::to_string(v.size()));

    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id   = "map";
    pose.pose.position.x   = v[0];
    pose.pose.position.y   = v[1];
    pose.pose.position.z   = (v.size() == 7) ? v[2] : 0.0;

    if (v.size() == 7) {
        // Full quaternion supplied
        pose.pose.orientation.x = v[3];
        pose.pose.orientation.y = v[4];
        pose.pose.orientation.z = v[5];
        pose.pose.orientation.w = v[6];
    } else if (v.size() == 3) {
        // yaw (radians) → quaternion
        const double half_yaw = v[2] * 0.5;
        pose.pose.orientation.w = std::cos(half_yaw);
        pose.pose.orientation.z = std::sin(half_yaw);
        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
    } else {
        // identity orientation
        pose.pose.orientation.w = 1.0;
    }

    return pose;
}
}  // namespace BT

namespace sp_decision
{

/**
 * @class NavToPose
 * @brief BehaviorTree StatefulActionNode that drives the robot to a target pose
 *        by calling the "navigate_to_pose" ROS2 action server.
 *
 * Input Ports  (至少提供其中一个)
 * ───────────
 *   goal_pose         (std::string)               – 2-D 坐标字符串: "(x,y)"
 *                                                    e.g. "(10.0,9.0)"
 *   goal_pose_stamped (geometry_msgs/PoseStamped) – 直接从 blackboard 取 PoseStamped
 *                                                    在 XML 中写  goal_pose_stamped="{bb_key}"
 *
 * Blackboard Keys (read)
 * ──────────────────────
 *   node  (rclcpp::Node::SharedPtr)  – shared ROS2 node used to create the
 *                                      action client and spin callbacks
 *
 * Return Codes
 * ────────────
 *   RUNNING   – waiting for the action server to accept / execute the goal
 *   SUCCESS   – the action server reported SUCCEEDED
 *   FAILURE   – the action server rejected, aborted, or an error occurred
 */
class NavToPose : public BT::StatefulActionNode
{
public:
  using NavigateToPose = robot_msg::action::NavigateToPose;
  using GoalHandle     = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  /**
   * @brief Constructor – retrieves the shared node from the blackboard and
   *        sets up the dedicated action-client callback group / executor.
   */
  NavToPose(const std::string & name, const BT::NodeConfiguration & config);

  ~NavToPose() override = default;

  /** @brief Declare the input ports consumed by this node. */
  static BT::PortsList providedPorts();

  // ── StatefulActionNode interface ───────────────────────────────────────────

  /**
   * @brief Called once when the node transitions from IDLE to RUNNING.
   *        Sends the goal to the action server.
   * @return RUNNING if the goal was sent, FAILURE otherwise.
   */
  BT::NodeStatus onStart() override;

  /**
   * @brief Called every tick while the node is in the RUNNING state.
   *        Spins the dedicated executor to process pending callbacks and
   *        checks whether the action has completed.
   * @return RUNNING / SUCCESS / FAILURE depending on action state.
   */
  BT::NodeStatus onRunning() override;

  /**
   * @brief Called when the BT requests a halt (e.g. a parent Sequence fails).
   *        Cancels the active action goal if one exists.
   */
  void onHalted() override;

private:
  // ── Internal action state ─────────────────────────────────────────────────
  enum class ActionState : int
  {
    IDLE      = 0,
    PENDING   = 1,   ///< goal sent, waiting for acceptance
    RUNNING   = 2,   ///< goal accepted, robot moving
    SUCCEEDED = 3,
    FAILED    = 4,
    CANCELED  = 5,
  };

  // ── ROS2 resources ────────────────────────────────────────────────────────
  rclcpp::Node::SharedPtr                            node_;      ///< private node, not shared with main executor
  rclcpp::executors::SingleThreadedExecutor          executor_;  ///< sole owner of node_
  rclcpp_action::Client<NavigateToPose>::SharedPtr   action_client_;

  // ── Goal tracking ─────────────────────────────────────────────────────────
  GoalHandle::SharedPtr   goal_handle_;
  std::atomic<int>        action_state_{static_cast<int>(ActionState::IDLE)};
  std::atomic<uint64_t>   goal_gen_{0};   ///< incremented on each halt/start to discard stale callbacks

  // ── Action callbacks ──────────────────────────────────────────────────────
  void goal_response_cb(const GoalHandle::SharedPtr & goal_handle);

  void feedback_cb(
    GoalHandle::SharedPtr /*handle*/,
    const std::shared_ptr<const NavigateToPose::Feedback> feedback);

  void result_cb(const GoalHandle::WrappedResult & result);

  /**
   * @brief Parse a coordinate string of the form "(x,y)" or "(x,y,yaw)"
   *        into a PoseStamped (frame_id = "map", yaw in radians).
   * @return Populated PoseStamped, or std::nullopt on parse failure.
   */
  static std::optional<geometry_msgs::msg::PoseStamped>
    parse_goal_string(const std::string & s);};

}  // namespace sp_decision

#endif  // SP_DECISION_PLUGIN_NAV_TO_POSE_HPP_
