#ifndef SP_NAV_BT_PLUGINS_NAV_ACTION_BASE_HPP_
#define SP_NAV_BT_PLUGINS_NAV_ACTION_BASE_HPP_

#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace sp_nav_bt
{

/**
 * @brief Base class for simple navigation actions.
 * T is the ROS action type.
 */
template <typename ActionT>
class NavActionBase : public BT::CoroActionNode
{
public:
  NavActionBase(const std::string& name, const BT::NodeConfiguration& conf)
    : BT::CoroActionNode(name, conf)
  {
    if (!config().blackboard->template get<rclcpp::Node::SharedPtr>("node", node_)) {
        throw std::runtime_error("NavActionBase: rclcpp::Node::SharedPtr missing in blackboard");
    }
  }

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<std::string>("server_name") };
  }

protected:
  rclcpp::Node::SharedPtr node_;
  typename rclcpp_action::Client<ActionT>::SharedPtr action_client_;
};

} // namespace sp_nav_bt

#endif // SP_NAV_BT_PLUGINS_NAV_ACTION_BASE_HPP_
