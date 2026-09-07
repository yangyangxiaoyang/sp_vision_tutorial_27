#ifndef SP_NAV_BT_PLUGINS_COMPUTE_PATH_SERVICE_HPP_
#define SP_NAV_BT_PLUGINS_COMPUTE_PATH_SERVICE_HPP_

#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/srv/get_plan.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

namespace sp_nav_bt
{

/**
 * @brief Action node to call a global planning service (nav_msgs/srv/GetPlan).
 */
class ComputePathService : public BT::CoroActionNode
{
public:
  ComputePathService(const std::string& name, const BT::NodeConfiguration& conf);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

  void halt() override;

protected:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<nav_msgs::srv::GetPlan>::SharedPtr service_client_;
};

} // namespace sp_nav_bt

#endif // SP_NAV_BT_PLUGINS_COMPUTE_PATH_SERVICE_HPP_
