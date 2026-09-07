#include "sp_nav_bt/plugins/get_pose.hpp"

#include "behaviortree_cpp_v3/bt_factory.h"
#include "tf2/exceptions.h"
#include "tf2/time.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace sp_nav_bt
{

GetPose::GetPose(const std::string & name, const BT::NodeConfiguration & conf)
: BT::SyncActionNode(name, conf)
{
  if (!config().blackboard->template get<rclcpp::Node::SharedPtr>("node", node_)) {
    throw std::runtime_error("GetPose: rclcpp::Node::SharedPtr missing in blackboard");
  }

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

BT::PortsList GetPose::providedPorts()
{
  return {
    BT::InputPort<std::string>("parent_frame", "map",       "Reference (parent) coordinate frame"),
    BT::InputPort<std::string>("child_frame",  "base_link", "Target (child) coordinate frame"),
    BT::InputPort<double>     ("timeout_ms",   100.0,       "TF wait timeout in milliseconds"),
    BT::OutputPort<geometry_msgs::msg::PoseStamped>("pose", "Pose of child_frame expressed in parent_frame")
  };
}

BT::NodeStatus GetPose::tick()
{
  std::string parent_frame, child_frame;
  double timeout_ms = 100.0;

  getInput("parent_frame", parent_frame);
  getInput("child_frame",  child_frame);
  getInput("timeout_ms",   timeout_ms);

  try {
    // Wait up to timeout_ms for the transform to become available.
    if (!tf_buffer_->canTransform(
        parent_frame, child_frame, tf2::TimePointZero,
        tf2::durationFromSec(timeout_ms / 1000.0)))
    {
      RCLCPP_WARN(node_->get_logger(),
        "[GetPose] Transform %s -> %s not available within %.0f ms",
        parent_frame.c_str(), child_frame.c_str(), timeout_ms);
      return BT::NodeStatus::FAILURE;
    }

    auto transform = tf_buffer_->lookupTransform(
      parent_frame, child_frame, tf2::TimePointZero);

    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = parent_frame;
    pose.header.stamp    = node_->now();
    pose.pose.position.x = transform.transform.translation.x;
    pose.pose.position.y = transform.transform.translation.y;
    pose.pose.position.z = transform.transform.translation.z;
    pose.pose.orientation = transform.transform.rotation;

    setOutput("pose", pose);

    RCLCPP_DEBUG(node_->get_logger(),
      "[GetPose] %s -> %s: (%.3f, %.3f, %.3f)",
      parent_frame.c_str(), child_frame.c_str(),
      pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);

    return BT::NodeStatus::SUCCESS;

  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(node_->get_logger(),
      "[GetPose] TF lookup failed (%s -> %s): %s",
      parent_frame.c_str(), child_frame.c_str(), ex.what());
    return BT::NodeStatus::FAILURE;
  }
}

}  // namespace sp_nav_bt

// Plugin registration
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<sp_nav_bt::GetPose>("nav_GetPose");
}
