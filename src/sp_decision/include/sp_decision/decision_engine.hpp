#ifndef SP_DECISION_DECISION_ENGINE_HPP_
#define SP_DECISION_DECISION_ENGINE_HPP_

#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sp_decision/bt_engine.hpp"

namespace sp_decision
{

/**
 * @class DecisionEngine
 * @brief Manages continuous BT execution for sp_decision.
 *
 * Features
 * --------
 * - Always ticks the active tree at a configurable period.
 * - When the tree returns SUCCESS / FAILURE it is automatically reset and
 *   re-ticked so the behaviour is truly continuous.
 * - Supports hot-switching to a different XML file while running.
 * - Logger flags (cout / file / ZMQ) can be configured before start() or
 *   dynamically; the change takes effect on the next tree (re)initialisation.
 */
class DecisionEngine
{
public:
  /**
   * @param node          Shared ROS2 node (used by BT actions / services).
   * @param tick_period   Default tick period (can be overridden in start()).
   */
  explicit DecisionEngine(
    rclcpp::Node::SharedPtr node,
    std::chrono::milliseconds tick_period = std::chrono::milliseconds(50));

  ~DecisionEngine();

  // ── Plugin management ──────────────────────────────────────────────────
  /**
   * @brief Set BehaviourTree plugin libraries to load.
   *        Must be called before start().
   */
  void set_plugin_libraries(const std::vector<std::string> & libs);

  // ── Tree file management ───────────────────────────────────────────────
  /**
   * @brief Register available BT XML files.
   *        Populates the list shown by the GUI.
   */
  void set_tree_files(const std::vector<std::string> & xml_paths);

  /** @brief Return the registered tree file list. */
  const std::vector<std::string> & get_tree_files() const { return tree_files_; }

  // ── Lifecycle ─────────────────────────────────────────────────────────
  /**
   * @brief Load plugins, initialise the first tree and start the tick thread.
   * @param xml_file      Path to the initial XML file (empty = first in list).
   */
  bool start(const std::string & xml_file = "");

  /** @brief Request the tick thread to stop and join. */
  void stop();

  /** @brief Switch to a different BT XML file while the engine is running. */
  bool switch_tree(const std::string & xml_file);

  bool is_running() const { return running_.load(); }
  const std::string & current_tree_file() const { return current_tree_file_; }

  // ── Blackboard access ─────────────────────────────────────────────────
  BT::Blackboard::Ptr get_blackboard();

  // ── Logger toggles ────────────────────────────────────────────────────
  void set_cout_logger(bool enable);
  void set_file_logger(bool enable, const std::string & path = "bt_trace.fbl");
  void set_zmq_publisher(bool enable);

  // ── Status callback ───────────────────────────────────────────────────
  /**
   * @brief Optional callback invoked from the tick thread on status events.
   *        The string argument carries a human-readable message.
   *        Will be called from a background thread – use Qt::QueuedConnection
   *        or a mutex-protected queue in the GUI.
   */
  std::function<void(const std::string &)> on_status_update;

  /**
   * @brief Called (from the tick thread) every time a tree is successfully
   *        (re)initialised – useful for re-binding the BlackboardBridge
   *        without conflicting with the GUI's on_status_update.
   */
  std::function<void()> on_tree_initialized;

private:
  void run_loop();
  bool reinit_tree(const std::string & xml_file);

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<BtEngine> engine_;

  std::vector<std::string> plugin_libs_;
  std::vector<std::string> tree_files_;
  std::string current_tree_file_;

  std::thread bt_thread_;
  std::atomic<bool> running_{false};
  std::atomic<bool> stop_requested_{false};

  // Hot-switch
  std::string pending_tree_file_;
  std::atomic<bool> switch_requested_{false};
  std::mutex switch_mutex_;

  // Logger flags (applied on next init_tree)
  bool cout_logger_enabled_   = true;
  bool file_logger_enabled_   = false;
  bool zmq_publisher_enabled_ = false;
  std::string file_logger_path_ = "bt_trace.fbl";

  std::chrono::milliseconds tick_period_;
};

}  // namespace sp_decision

#endif  // SP_DECISION_DECISION_ENGINE_HPP_
