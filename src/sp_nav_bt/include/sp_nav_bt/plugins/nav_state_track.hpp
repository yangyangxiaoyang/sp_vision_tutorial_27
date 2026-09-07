#ifndef SP_NAV_BT_PLUGINS_NAV_STATE_TRACK_HPP_
#define SP_NAV_BT_PLUGINS_NAV_STATE_TRACK_HPP_

#include <chrono>
#include <memory>
#include <string>

#include "behaviortree_cpp_v3/action_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace sp_nav_bt
{

/**
 * @brief StatefulActionNode that monitors navigation progress.
 *
 * On each tick it:
 *   1. Looks up the map → base_link TF to obtain the current robot pose.
 *   2. Computes the Euclidean distance to the goal pose.
 *   3. Writes "remaining_distance" (double, metres) and
 *      "elapsed_time" (double, seconds) to the blackboard.
 *   4. Returns SUCCESS when remaining_distance < goal_tolerance,
 *      RUNNING otherwise.
 *
 * Ports
 * ─────
 *   Input  : goal_pose        – geometry_msgs/PoseStamped  (default: read from BB key "goal_pose")
 *   Input  : goal_tolerance   – double, metres              (default: 0.2)
 *   Output : remaining_distance – double, metres
 *   Output : elapsed_time       – double, seconds
 */
class NavStateTrack : public BT::StatefulActionNode
{
public:
  NavStateTrack(const std::string & name, const BT::NodeConfiguration & conf);

  static BT::PortsList providedPorts();

  /// Called once when the node transitions from IDLE → RUNNING.
  BT::NodeStatus onStart() override;

  /// Called on every tick while the node is RUNNING.
  BT::NodeStatus onRunning() override;

  /// Called when the node is halted externally.
  void onHalted() override;

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::chrono::steady_clock::time_point start_time_;
};

}  // namespace sp_nav_bt

#endif  // SP_NAV_BT_PLUGINS_NAV_STATE_TRACK_HPP_
