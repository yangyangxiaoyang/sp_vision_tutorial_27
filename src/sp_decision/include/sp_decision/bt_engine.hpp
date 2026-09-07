#ifndef SP_DECISION_BT_ENGINE_HPP_
#define SP_DECISION_BT_ENGINE_HPP_

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/loggers/bt_cout_logger.h"
#include "behaviortree_cpp_v3/loggers/bt_file_logger.h"
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"
#include "behaviortree_cpp_v3/utils/shared_library.h"
#include "behaviortree_cpp_v3/xml_parsing.h"
#include "rclcpp/rclcpp.hpp"

namespace sp_decision
{

/**
 * @enum BtStatus
 * @brief Result of a BT execution run.
 */
enum class BtStatus
{
  SUCCEEDED,
  FAILED,
  CANCELED,
  RUNNING
};

/**
 * @class BtEngine
 * @brief Self-contained BehaviorTree engine for sp_decision.
 *
 * Features (independent of sp_nav_bt)
 * ─────────────────────────────────────
 * - Plugin loading via BT::SharedLibrary.
 * - Conditional logger creation: StdCout / file / ZMQ (Groot).
 * - tick_once() for external per-tick control.
 * - halt() to interrupt a running tree.
 * - Blackboard shared with all tree nodes.
 */
class BtEngine
{
public:
  explicit BtEngine(rclcpp::Node::SharedPtr node);
  virtual ~BtEngine() = default;

  // ── Plugin management ──────────────────────────────────────────────────
  /**
   * @brief Load BT plugin shared libraries.
   * @param libs Library names (e.g. "compute_path_service").
   */
  void load_plugins(const std::vector<std::string> & libs);

  // ── Tree initialisation ────────────────────────────────────────────────
  /**
   * @brief Create a BT from an XML file and (re)initialise all active loggers.
   *        Calling this a second time replaces the previous tree.
   */
  bool init_tree(const std::string & xml_file);

  // ── Execution ─────────────────────────────────────────────────────────
  /**
   * @brief Execute one tick and return the raw BT node status.
   *        The is_halted_ flag is cleared before the first tick after a halt.
   */
  BT::NodeStatus tick_once();

  /**
   * @brief Blocking run loop: ticks until the tree finishes or is halted.
   * @param loop_rate  Tick period.
   */
  BtStatus run(std::chrono::milliseconds loop_rate = std::chrono::milliseconds(100));

  /**
   * @brief Request the tree to halt on the next tick boundary.
   */
  void halt();

  // ── Blackboard access ─────────────────────────────────────────────────
  BT::Blackboard::Ptr get_blackboard() { return blackboard_; }

  // ── Logger toggles (take effect on next init_tree call) ───────────────
  void set_cout_logger_enabled(bool enable)                              { enable_cout_logger_ = enable; }
  void set_file_logger_enabled(bool enable, const std::string & path = "bt_trace.fbl")
  {
    enable_file_logger_ = enable;
    if (!path.empty()) { file_logger_path_ = path; }
  }
  void set_zmq_publisher_enabled(bool enable)                           { enable_zmq_publisher_ = enable; }

  bool get_cout_logger_enabled()   const { return enable_cout_logger_; }
  bool get_file_logger_enabled()   const { return enable_file_logger_; }
  bool get_zmq_publisher_enabled() const { return enable_zmq_publisher_; }

protected:
  rclcpp::Node::SharedPtr node_;

  BT::BehaviorTreeFactory factory_;
  BT::Tree                tree_;
  BT::Blackboard::Ptr     blackboard_;

  std::unique_ptr<BT::StdCoutLogger> cout_logger_;
  std::unique_ptr<BT::FileLogger>    file_logger_;
  std::unique_ptr<BT::PublisherZMQ>  zmq_publisher_;

  bool is_halted_ = false;

  // Logger flags
  bool        enable_cout_logger_   = true;
  bool        enable_file_logger_   = false;
  bool        enable_zmq_publisher_ = false;
  std::string file_logger_path_     = "bt_trace.fbl";
};

}  // namespace sp_decision

#endif  // SP_DECISION_BT_ENGINE_HPP_
