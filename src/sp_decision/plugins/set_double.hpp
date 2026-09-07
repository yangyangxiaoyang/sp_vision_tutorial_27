#ifndef SP_DECISION_PLUGIN_SET_DOUBLE_HPP_
#define SP_DECISION_PLUGIN_SET_DOUBLE_HPP_

#include <string>

#include "behaviortree_cpp_v3/action_node.h"

namespace sp_decision
{

/**
 * @class SetDouble
 * @brief BehaviorTree SyncActionNode that writes a double value to a blackboard
 *        key and immediately returns SUCCESS.
 *
 * Input Ports
 * ───────────
 *   key    (std::string)  目标 blackboard key 名称
 *   value  (double)       写入的浮点值
 *
 * XML example
 * ───────────
 *   <SetDouble key="target_distance" value="5.0"/>
 *   <SetDouble key="wait_time"       value="{some_double_var}"/>
 */
class SetDouble : public BT::SyncActionNode
{
public:
  SetDouble(const std::string & name, const BT::NodeConfiguration & config)
      : BT::SyncActionNode(name, config) {}

  ~SetDouble() override = default;

  static BT::PortsList providedPorts()
  {
    return {
        BT::InputPort<std::string>("key",   "目标 blackboard key 名称"),
        BT::InputPort<double>     ("value", 0.0, "写入的浮点值"),
    };
  }

  BT::NodeStatus tick() override
  {
    std::string key;
    double value = 0.0;

    if (!getInput("key", key) || key.empty())
    {
      return BT::NodeStatus::FAILURE;
    }
    getInput("value", value);

    config().blackboard->set<double>(key, value);
    return BT::NodeStatus::SUCCESS;
  }
};

}  // namespace sp_decision

#endif  // SP_DECISION_PLUGIN_SET_DOUBLE_HPP_
