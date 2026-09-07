#include "sp_nav_bt/plugins/nav_state_track.hpp"

#include <cmath>

#include "behaviortree_cpp_v3/bt_factory.h"
#include "tf2/exceptions.h"

namespace sp_nav_bt
{

NavStateTrack::NavStateTrack(const std::string & name, const BT::NodeConfiguration & conf)
: BT::StatefulActionNode(name, conf)
{
  if (!config().blackboard->template get<rclcpp::Node::SharedPtr>("node", node_)) {
    throw std::runtime_error("NavStateTrack: rclcpp::Node::SharedPtr missing in blackboard");
  }

  // Create TF components once; they are re-used across ticks.
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

BT::PortsList NavStateTrack::providedPorts()
{
  return {
    BT::InputPort<geometry_msgs::msg::PoseStamped>(
      "goal_pose", "Target pose (default: read from blackboard key 'goal_pose')"),
    BT::InputPort<double>(
      "goal_tolerance", 0.2, "Distance threshold to consider goal reached (metres)"),
    BT::OutputPort<double>("remaining_distance", "Euclidean distance to goal (metres)"),
    BT::OutputPort<double>("elapsed_time",       "Time elapsed since task started (seconds)")
  };
}

// ─── Lifecycle callbacks ───────────────────────────────────────────────────────

BT::NodeStatus NavStateTrack::onStart()
{
  start_time_ = std::chrono::steady_clock::now();
  RCLCPP_INFO(node_->get_logger(), "[NavStateTrack] Task started, monitoring navigation progress.");
  RCLCPP_INFO(node_->get_logger(), "[NavStateTrack] onStart() entered.");
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus NavStateTrack::onRunning()
{
  RCLCPP_INFO(node_->get_logger(), "[NavStateTrack] onRunning() ticked.");

  // ── 1. Get goal pose ──────────────────────────────────────────────────────
  geometry_msgs::msg::PoseStamped goal_pose;
  if (!getInput<geometry_msgs::msg::PoseStamped>("goal_pose", goal_pose)) {
    // Fall back to reading directly from the blackboard key set by NavInterfaceNode.
    if (!config().blackboard->template get<geometry_msgs::msg::PoseStamped>("goal_pose", goal_pose)) {
      RCLCPP_WARN(node_->get_logger(), "[NavStateTrack] goal_pose not available yet, RUNNING.");
      return BT::NodeStatus::RUNNING;
    }
  }

  // ── 2. Compute elapsed time ───────────────────────────────────────────────
  const double elapsed_sec =
    std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time_).count();

  // ── 3. Look up current robot pose (map → base_link) ───────────────────────
  double current_x = 0.0;
  double current_y = 0.0;
  try {
    auto transform = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
    current_x = transform.transform.translation.x;
    current_y = transform.transform.translation.y;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(node_->get_logger(),
      "[NavStateTrack] TF lookup failed: %s. Will retry.", ex.what());
    // Write what we have and keep RUNNING.
    setOutput("elapsed_time", elapsed_sec);
    config().blackboard->set<double>("elapsed_time", elapsed_sec);
    return BT::NodeStatus::RUNNING;
  }

  // ── 4. Compute remaining distance (2-D Euclidean) ─────────────────────────
  const double dx = goal_pose.pose.position.x - current_x;
  const double dy = goal_pose.pose.position.y - current_y;
  const double remaining = std::hypot(dx, dy);

  // ── 5. Write to ports and blackboard ─────────────────────────────────────
  setOutput("remaining_distance", remaining);
  setOutput("elapsed_time",       elapsed_sec);

  config().blackboard->set<double>("remaining_distance", remaining);
  config().blackboard->set<double>("elapsed_time",       elapsed_sec);

  RCLCPP_DEBUG(node_->get_logger(),
    "[NavStateTrack] remaining=%.3f m  elapsed=%.2f s  pos=(%.2f, %.2f)",
    remaining, elapsed_sec, current_x, current_y);

  // ── 6. Check arrival ─────────────────────────────────────────────────────
  double tolerance = 0.2;
  getInput<double>("goal_tolerance", tolerance);

  RCLCPP_INFO(node_->get_logger(),
    "[NavStateTrack] goal=(%.2f, %.2f) current=(%.2f, %.2f) remaining=%.3f tol=%.3f",
    goal_pose.pose.position.x, goal_pose.pose.position.y,
    current_x, current_y,
    remaining, tolerance);

  if (remaining < tolerance) {
    RCLCPP_INFO(node_->get_logger(),
      "[NavStateTrack] Goal reached! remaining=%.3f m, elapsed=%.2f s", remaining, elapsed_sec);
    return BT::NodeStatus::SUCCESS;
  }

  RCLCPP_INFO(node_->get_logger(),
    "[NavStateTrack] Goal not reached yet, returning RUNNING.");

  return BT::NodeStatus::RUNNING;
}

void NavStateTrack::onHalted()
{
  setStatus(BT::NodeStatus::IDLE);
  RCLCPP_INFO(node_->get_logger(), "[NavStateTrack] Halted externally.");
}

}  // namespace sp_nav_bt

// Plugin registration
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<sp_nav_bt::NavStateTrack>("nav_NavStateTrack");
}
