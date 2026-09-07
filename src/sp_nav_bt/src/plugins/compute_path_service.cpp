#include "sp_nav_bt/plugins/compute_path_service.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"

namespace sp_nav_bt
{

ComputePathService::ComputePathService(const std::string& name, const BT::NodeConfiguration& conf)
  : BT::CoroActionNode(name, conf)
{
  if (!config().blackboard->template get<rclcpp::Node::SharedPtr>("node", node_)) {
    throw std::runtime_error("ComputePathService: rclcpp::Node::SharedPtr missing in blackboard");
  }

  std::string service_name;
  if (!getInput<std::string>("service_name", service_name)) {
    service_name = "make_plan"; // Default service name as requested
  }
  
  service_client_ = node_->create_client<nav_msgs::srv::GetPlan>(service_name);
}

BT::PortsList ComputePathService::providedPorts()
{
  return {
    BT::InputPort<std::string>("service_name", "make_plan", "Name of the service"),
    BT::InputPort<geometry_msgs::msg::PoseStamped>("start", "Starting pose"),
    BT::InputPort<geometry_msgs::msg::PoseStamped>("goal", "Goal pose"),
    BT::OutputPort<nav_msgs::msg::Path>("path", "The computed global path")
  };
}

BT::NodeStatus ComputePathService::tick()
{
  // 1. Check if service is available
  if (!service_client_->service_is_ready()) {
    if (!service_client_->wait_for_service(std::chrono::milliseconds(100))) {
      RCLCPP_WARN(node_->get_logger(), "Service %s not ready", service_client_->get_service_name());
      return BT::NodeStatus::FAILURE;
    }
  }

  // 2. Prepare request
  auto request = std::make_shared<nav_msgs::srv::GetPlan::Request>();
  if (!getInput<geometry_msgs::msg::PoseStamped>("start", request->start) ||
      !getInput<geometry_msgs::msg::PoseStamped>("goal", request->goal)) {
    RCLCPP_ERROR(node_->get_logger(), "ComputePathService: start or goal pose missing");
    return BT::NodeStatus::FAILURE;
  }
  
  request->tolerance = 0.5; // Optional tolerance

  // RCLCPP_INFO(node_->get_logger(), "Sending request to %s", service_client_->get_service_name());

  // 3. Send request asynchronously
  auto future_result = service_client_->async_send_request(request);

  // 4. Yield and wait for response
  // In CoroActionNode, we can yield while waiting
  while (future_result.wait_for(std::chrono::milliseconds(10)) != std::future_status::ready) {
    setStatusRunningAndYield();
  }

  // 5. Handle response
  auto result = future_result.get();
  if (result && !result->plan.poses.empty()) {
    // RCLCPP_INFO(node_->get_logger(), "ComputePathService: Path found with %zu poses", result->plan.poses.size());
    setOutput("path", result->plan);
    return BT::NodeStatus::SUCCESS;
  } else {
    RCLCPP_ERROR(node_->get_logger(), "ComputePathService: Failed to get plan or plan is empty");
    return BT::NodeStatus::FAILURE;
  }
}

void ComputePathService::halt()
{
  BT::CoroActionNode::halt();
}

} // namespace sp_nav_bt

// Pluging registration
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<sp_nav_bt::ComputePathService>("nav_ComputePathService");
}
