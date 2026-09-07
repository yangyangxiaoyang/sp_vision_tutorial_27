#include "sp_nav_bt/nav_bt_engine.hpp"

#include <iostream>
#include <memory>
#include <string>
#include <vector>

namespace sp_nav_bt
{

NavBTEngine::NavBTEngine(rclcpp::Node::SharedPtr node)
: node_(node)
{
  blackboard_ = BT::Blackboard::create();
  // Store node pointer in blackboard for custom nodes to use
  blackboard_->set<rclcpp::Node::SharedPtr>("node", node_);

  node_->declare_parameter<bool>("enable_bt_cout_logger", false);
}

void NavBTEngine::load_plugins(const std::vector<std::string> &plugin_libraries)
{
  BT::SharedLibrary loader;
  for (const auto &lib_name : plugin_libraries)
  {
    RCLCPP_INFO(node_->get_logger(), "Loading plugin: %s", lib_name.c_str());
    try
    {
      // Try loading with the standardized OS name (libXXXX.so)
      factory_.registerFromPlugin(loader.getOSName(lib_name));
    }
    catch (const std::exception &ex)
    {
       RCLCPP_WARN(node_->get_logger(), "Failed to load via OSName, trying literal: %s", lib_name.c_str());
       try {
         // Some environments need the full "libXXXX.so" name directly if not in LD_LIBRARY_PATH
         factory_.registerFromPlugin(lib_name);
       } catch (const std::exception &ex2) {
         RCLCPP_ERROR(node_->get_logger(), "Failed to load BT plugin %s: %s", lib_name.c_str(), ex2.what());
       }
    }
  }
}

bool NavBTEngine::init_tree(const std::string &xml_file)
{
  RCLCPP_INFO(node_->get_logger(), "Initializing BT from: %s", xml_file.c_str());
  try
  {
    tree_ = factory_.createTreeFromFile(xml_file, blackboard_);
    
    // Create loggers
    if (node_->get_parameter("enable_bt_cout_logger").as_bool()) {
      std_cout_logger_ = std::make_unique<BT::StdCoutLogger>(tree_);
      RCLCPP_INFO(node_->get_logger(), "BT StdCoutLogger enabled.");
    }
    // file_logger_ = std::make_unique<BT::FileLogger>(tree_, "bt_trace.fbl");
    // zmq_publisher_ = std::make_unique<BT::PublisherZMQ>(tree_);

    return true;
  }
  catch (const std::exception &ex)
  {
    RCLCPP_ERROR(node_->get_logger(), "BehaviorTree initialization error: %s", ex.what());
    return false;
  }
}

BtStatus NavBTEngine::run(std::chrono::milliseconds loop_rate_ms)
{
  rclcpp::WallRate loop_rate(loop_rate_ms);
  BT::NodeStatus status = BT::NodeStatus::RUNNING;

  is_halted_ = false;

  // IMPORTANT: Reset the state of all nodes in the tree so it starts from scratch
  // since the tree object is held statically in memory.
  tree_.rootNode()->halt();

  RCLCPP_INFO(node_->get_logger(), "Running Behavior Tree...");

  try
  {
    while (rclcpp::ok() && status == BT::NodeStatus::RUNNING && !is_halted_)
    {
      status = tree_.tickRoot();
      
      if (!loop_rate.sleep())
      {
        //  RCLCPP_WARN(node_->get_logger(), "BT tick rate %0.2f Hz exceeded!", 
        //             1.0 / (loop_rate.period().count() * 1e-9));
      }
    }
  }
  catch (const std::exception &ex)
  {
      RCLCPP_ERROR(node_->get_logger(), "BT execution exception: %s", ex.what());
      return BtStatus::FAILED;
  }

  if (is_halted_) return BtStatus::CANCELED;
  return (status == BT::NodeStatus::SUCCESS) ? BtStatus::SUCCEEDED : BtStatus::FAILED;
}

void NavBTEngine::halt()
{
  is_halted_ = true;
  tree_.rootNode()->halt();
}

} // namespace sp_nav_bt
