#ifndef SP_NAV_BT_PLUGINS_GET_POSE_HPP_
#define SP_NAV_BT_PLUGINS_GET_POSE_HPP_

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
 * @brief SyncActionNode that performs a single TF lookup and writes the resulting
 *        PoseStamped into the blackboard output port.
 *
 * Ports
 * ─────
 *   Input  : parent_frame  – string, the reference (parent) frame  (default: "map")
 *   Input  : child_frame   – string, the target (child) frame       (default: "base_link")
 *   Input  : timeout_ms    – double, TF wait timeout in milliseconds (default: 100.0)
 *   Output : pose          – geometry_msgs/PoseStamped retrieved from TF
 */
class GetPose : public BT::SyncActionNode
{
public:
  GetPose(const std::string & name, const BT::NodeConfiguration & conf);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace sp_nav_bt

#endif  // SP_NAV_BT_PLUGINS_GET_POSE_HPP_
