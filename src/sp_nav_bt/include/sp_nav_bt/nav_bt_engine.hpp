#ifndef SP_NAV_BT_NAV_BT_ENGINE_HPP_
#define SP_NAV_BT_NAV_BT_ENGINE_HPP_

#include <iostream>
#include <memory>
#include <string>
#include <vector>
#include <chrono>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/xml_parsing.h"
#include "behaviortree_cpp_v3/loggers/bt_cout_logger.h"
#include "behaviortree_cpp_v3/loggers/bt_file_logger.h"
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"
#include "behaviortree_cpp_v3/utils/shared_library.h"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace sp_nav_bt
{

/**
 * @enum BtStatus
 * @brief Represents the current status of BT execution.
 */
enum class BtStatus
{
  SUCCEEDED,
  FAILED,
  CANCELED,
  RUNNING
};

/**
 * @class NavBTEngine
 * @brief Manages the Behavior Tree life cycle for navigation tasks.
 */
class NavBTEngine
{
public:
  explicit NavBTEngine(rclcpp::Node::SharedPtr node);
  virtual ~NavBTEngine() = default;

  /**
   * @brief Load BT plugin libraries.
   * @param plugin_libraries A vector of library names (e.g., "libnav_actions.so").
   */
  void load_plugins(const std::vector<std::string> &plugin_libraries);

  /**
   * @brief Load a specific BT XML file.
   * @param xml_file Path to the behavior tree XML.
   */
  bool init_tree(const std::string &xml_file);

  /**
   * @brief Execute the current behavior tree until completion or interruption.
   * @param loop_period_ms Tick period in milliseconds.
   */
  BtStatus run(std::chrono::milliseconds loop_rate = std::chrono::milliseconds(100));

  /**
   * @brief Manually halt the execution.
   */
  void halt();

  /**
   * @brief Get the internal blackboard.
   */
  BT::Blackboard::Ptr get_blackboard() { return blackboard_; }

protected:
  rclcpp::Node::SharedPtr node_;
  BT::BehaviorTreeFactory factory_;
  BT::Tree tree_;
  BT::Blackboard::Ptr blackboard_;
  
  std::unique_ptr<BT::StdCoutLogger> std_cout_logger_;
  std::unique_ptr<BT::FileLogger> file_logger_;
  std::unique_ptr<BT::PublisherZMQ> zmq_publisher_;
  
  bool is_halted_ = false;
};

} // namespace sp_nav_bt

#endif // SP_NAV_BT_NAV_BT_ENGINE_HPP_
