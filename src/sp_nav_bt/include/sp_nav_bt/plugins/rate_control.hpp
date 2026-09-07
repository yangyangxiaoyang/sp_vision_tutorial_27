
#ifndef SP_DECISION_RATE_CONTROLLER_NODE
#define SP_DECISION_RATE_CONTROLLER_NODE

#include <chrono>
#include <string>

#include "behaviortree_cpp_v3/decorator_node.h"

/**
 * @brief A BT::DecoratorNode that ticks its child at a specified rate
 */
class RateControl : public BT::DecoratorNode
{
public:
  /**
   * @brief A constructor for nav2_behavior_tree::RateControl
   * @param name Name for the XML tag for this node
   * @param conf BT node configuration
   */
  RateControl(
      const std::string &name,
      const BT::NodeConfiguration &conf);

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return {
        BT::InputPort<double>("hz", 10.0, "Rate")};
  }

private:
  /**
   * @brief The main override required by a BT action
   * @return BT::NodeStatus Status of tick execution
   */
  BT::NodeStatus tick() override;

  std::chrono::time_point<std::chrono::high_resolution_clock> start_;
  double period_;
  bool first_time_;
};

#endif // SP_DECISION_RATE_CONTROLLER_NODE
