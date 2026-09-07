#include "sp_decision/decision_engine.hpp"

namespace sp_decision
{

// ─────────────────────────────────────────────────────────────────────────────

DecisionEngine::DecisionEngine(
  rclcpp::Node::SharedPtr node,
  std::chrono::milliseconds tick_period)
: node_(node), tick_period_(tick_period)
{}

DecisionEngine::~DecisionEngine()
{
  stop();
}

// ─────────────────────────────────────────────────────────────────────────────

void DecisionEngine::set_plugin_libraries(const std::vector<std::string> & libs)
{
  plugin_libs_ = libs;
}

void DecisionEngine::set_tree_files(const std::vector<std::string> & xml_paths)
{
  tree_files_ = xml_paths;
}

// ─────────────────────────────────────────────────────────────────────────────

bool DecisionEngine::start(const std::string & xml_file)
{
  if (running_.load()) {
    RCLCPP_WARN(node_->get_logger(), "[DecisionEngine] Already running.");
    return false;
  }

  std::string target = xml_file;
  if (target.empty() && !tree_files_.empty()) {
    target = tree_files_.front();
  }
  if (target.empty()) {
    RCLCPP_ERROR(node_->get_logger(), "[DecisionEngine] No BT XML file specified.");
    return false;
  }

  // Build a fresh engine
  engine_ = std::make_shared<BtEngine>(node_);
  engine_->set_cout_logger_enabled(cout_logger_enabled_);
  engine_->set_file_logger_enabled(file_logger_enabled_, file_logger_path_);
  engine_->set_zmq_publisher_enabled(zmq_publisher_enabled_);
  engine_->load_plugins(plugin_libs_);

  if (!engine_->init_tree(target)) {
    RCLCPP_ERROR(node_->get_logger(),
      "[DecisionEngine] Failed to init tree: %s", target.c_str());
    return false;
  }

  current_tree_file_ = target;
  if (on_tree_initialized) { on_tree_initialized(); }

  stop_requested_.store(false);
  switch_requested_.store(false);
  running_.store(true);

  bt_thread_ = std::thread(&DecisionEngine::run_loop, this);
  RCLCPP_INFO(node_->get_logger(),
    "[DecisionEngine] Started with tree: %s", target.c_str());
  return true;
}

// ─────────────────────────────────────────────────────────────────────────────

void DecisionEngine::stop()
{
  if (!running_.load()) { return; }

  stop_requested_.store(true);
  if (engine_) { engine_->halt(); }

  if (bt_thread_.joinable()) { bt_thread_.join(); }

  RCLCPP_INFO(node_->get_logger(), "[DecisionEngine] Stopped.");
}

// ─────────────────────────────────────────────────────────────────────────────

bool DecisionEngine::switch_tree(const std::string & xml_file)
{
  if (xml_file.empty() || !running_.load()) { return false; }

  std::lock_guard<std::mutex> lk(switch_mutex_);
  pending_tree_file_  = xml_file;
  switch_requested_.store(true);
  if (engine_) { engine_->halt(); }
  return true;
}

// ─────────────────────────────────────────────────────────────────────────────

BT::Blackboard::Ptr DecisionEngine::get_blackboard()
{
  if (!engine_) { return nullptr; }
  return engine_->get_blackboard();
}

// ─────────────────────────────────────────────────────────────────────────────

void DecisionEngine::set_cout_logger(bool enable)
{
  cout_logger_enabled_ = enable;
  if (engine_) { engine_->set_cout_logger_enabled(enable); }
}

void DecisionEngine::set_file_logger(bool enable, const std::string & path)
{
  file_logger_enabled_ = enable;
  if (!path.empty()) { file_logger_path_ = path; }
  if (engine_) { engine_->set_file_logger_enabled(enable, file_logger_path_); }
}

void DecisionEngine::set_zmq_publisher(bool enable)
{
  zmq_publisher_enabled_ = enable;
  if (engine_) { engine_->set_zmq_publisher_enabled(enable); }
}

// ─────────────────────────────────────────────────────────────────────────────

bool DecisionEngine::reinit_tree(const std::string & xml_file)
{
  engine_->set_cout_logger_enabled(cout_logger_enabled_);
  engine_->set_file_logger_enabled(file_logger_enabled_, file_logger_path_);
  engine_->set_zmq_publisher_enabled(zmq_publisher_enabled_);

  if (!engine_->init_tree(xml_file)) {
    RCLCPP_ERROR(node_->get_logger(),
      "[DecisionEngine] Failed to reinit tree: %s", xml_file.c_str());
    return false;
  }
  current_tree_file_ = xml_file;

  // Notify subscribers that a new tree is live
  if (on_tree_initialized) { on_tree_initialized(); }
  return true;
}

// ─────────────────────────────────────────────────────────────────────────────

void DecisionEngine::run_loop()
{
  rclcpp::WallRate rate(tick_period_);

  auto notify = [this](const std::string & msg)
  {
    RCLCPP_INFO(node_->get_logger(), "[DecisionEngine] %s", msg.c_str());
    if (on_status_update) { on_status_update(msg); }
  };

  notify("Started: " + current_tree_file_);

  while (rclcpp::ok() && !stop_requested_.load())
  {
    // ── Hot-switch processing ────────────────────────────────────────────
    if (switch_requested_.load())
    {
      std::string new_file;
      {
        std::lock_guard<std::mutex> lk(switch_mutex_);
        new_file = pending_tree_file_;
        switch_requested_.store(false);
      }

      const std::string prev = current_tree_file_;
      if (!reinit_tree(new_file)) {
        notify("ERROR loading " + new_file + " – reverting to " + prev);
        reinit_tree(prev);
      } else {
        notify("Switched to: " + new_file);
      }
    }

    // ── Single BT tick ───────────────────────────────────────────────────
    BT::NodeStatus status = BT::NodeStatus::RUNNING;
    try {
      status = engine_->tick_once();
    } catch (const std::exception & ex) {
      RCLCPP_ERROR(node_->get_logger(),
        "[DecisionEngine] tick exception: %s", ex.what());
      notify(std::string("EXCEPTION: ") + ex.what());
      rate.sleep();
      continue;
    }

    // ── Terminal states → reset node states and restart from root ────────
    // halt() calls tree_.rootNode()->halt(), resetting all nodes to IDLE.
    // The tree structure (XML) is NOT recreated; reinit_tree() is only
    // called when hot-switching to a different XML file.
    if (status == BT::NodeStatus::SUCCESS) {
      notify("SUCCEEDED – restarting");
      engine_->halt();
    } else if (status == BT::NodeStatus::FAILURE) {
      notify("FAILED – restarting");
      engine_->halt();
    }

    rate.sleep();
  }

  if (engine_) { engine_->halt(); }
  notify("Stopped.");
  running_.store(false);
}

}  // namespace sp_decision
