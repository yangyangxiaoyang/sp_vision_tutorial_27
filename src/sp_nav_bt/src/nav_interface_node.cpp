#include "sp_nav_bt/nav_interface_node.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace sp_nav_bt
{

NavInterfaceNode::NavInterfaceNode(const rclcpp::NodeOptions & options)
: Node("nav_interface_node", options)
{
  // 1. Declare parameters
  default_bt_xml_ = this->declare_parameter<std::string>("default_bt_xml", "point_to_point.xml");
  plugin_libs_ = this->declare_parameter<std::vector<std::string>>("bt_plugins", std::vector<std::string>());

  // 2. Initialize TF2
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

void NavInterfaceNode::init_engine()
{
  // 2. Initialize BT Engine (SAFE HERE AFTER CONSTRUCTOR)
  bt_engine_ = std::make_shared<NavBTEngine>(this->shared_from_this());
  bt_engine_->load_plugins(plugin_libs_);

  // 3. Initialize Tree ONCE
  if (!bt_engine_->init_tree(default_bt_xml_)) {
    RCLCPP_ERROR(this->get_logger(), "Failed to initial static BT: %s", default_bt_xml_.c_str());
  }

  // 4. Create Action Servers
  this->nav_to_pose_server_ = rclcpp_action::create_server<NavigateToPose>(
    this->get_node_base_interface(),
    this->get_node_clock_interface(),
    this->get_node_logging_interface(),
    this->get_node_waitables_interface(),
    "navigate_to_pose", // Action name
    std::bind(&NavInterfaceNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&NavInterfaceNode::handle_cancel, this, std::placeholders::_1),
    std::bind(&NavInterfaceNode::handle_accepted, this, std::placeholders::_1)
  );

  RCLCPP_INFO(this->get_logger(), "Navigation Interface Node initialized.");

  // Restart-task service：供 global_relocalization 重定位完成后调用，
  // halt 当前行为树并以相同目标点重新启动，BT 内部所有节点状态完全重置。
  restart_task_srv_ = this->create_service<std_srvs::srv::Trigger>(
    "/nav_interface/restart_task",
    std::bind(&NavInterfaceNode::handle_restart_task, this,
              std::placeholders::_1, std::placeholders::_2));
  RCLCPP_INFO(this->get_logger(),
    "Restart-task service ready at /nav_interface/restart_task");
}

rclcpp_action::GoalResponse NavInterfaceNode::handle_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const NavigateToPose::Goal> goal)
{
  (void)uuid;
  RCLCPP_INFO(this->get_logger(), "Received goal request: Go to (%.2f, %.2f)", 
    goal->pose.pose.position.x, goal->pose.pose.position.y);
  
  // Accept the goal
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse NavInterfaceNode::handle_cancel(
  const std::shared_ptr<GoalHandleNav> goal_handle)
{
  (void)goal_handle;
  RCLCPP_WARN(this->get_logger(), "Received cancel request");
  
  std::lock_guard<std::mutex> lock(bt_mutex_);
  if (is_task_running_) {
    bt_engine_->halt();
  }
  
  return rclcpp_action::CancelResponse::ACCEPT;
}

void NavInterfaceNode::handle_accepted(const std::shared_ptr<GoalHandleNav> goal_handle)
{
  // If there's already a task running, we preempt it (cancel previous)
  std::unique_lock<std::mutex> lock(bt_mutex_);
  if (is_task_running_) {
    RCLCPP_INFO(this->get_logger(), "Preempting previous task.");
    bt_engine_->halt();
    // Release the mutex BEFORE joining: execute_tree needs to acquire bt_mutex_
    // to finalize (set is_task_running_ = false). Joining while holding the lock
    // would cause a deadlock.
    lock.unlock();
    if (bt_thread_.joinable()) {
      bt_thread_.join();
    }
    lock.lock();
  }
  
  current_goal_handle_ = goal_handle;
  // 缓存 action 原始目标：后续地形节点会改写黑板 goal_pose，restart 必须用这份。
  original_goal_pose_ = goal_handle->get_goal()->pose;
  has_original_goal_pose_ = true;
  is_task_running_ = true;
  
  // Ensure previous thread is joined before assigning a new one
  if (bt_thread_.joinable()) {
    lock.unlock();
    bt_thread_.join();
    lock.lock();
  }

  // Start execution thread
  bt_thread_ = std::thread(&NavInterfaceNode::execute_tree, this);
}

void NavInterfaceNode::execute_tree()
{
  auto result = std::make_shared<NavigateToPose::Result>();

  // Use a local scope to handle completion so we can join correctly later if needed
  try {
    // 1. Prepare Blackboard with ORIGINAL goal（非地形中间点）
    auto blackboard = bt_engine_->get_blackboard();
    // 新任务强制失效 controller 使能缓存，确保首个 enable=true 会重新调服务同步服务端。
    // （抢占/取消可能在中断 pending 请求后仍残留 stale 的 true。）
    blackboard->set<bool>("controller_enabled", false);

    geometry_msgs::msg::PoseStamped goal_pose;
    {
      std::lock_guard<std::mutex> lock(bt_mutex_);
      if (has_original_goal_pose_) {
        goal_pose = original_goal_pose_;
      } else if (current_goal_handle_) {
        goal_pose = current_goal_handle_->get_goal()->pose;
        original_goal_pose_ = goal_pose;
        has_original_goal_pose_ = true;
      } else {
        RCLCPP_ERROR(this->get_logger(),
          "execute_tree: no original goal available, aborting.");
        is_task_running_ = false;
        return;
      }
    }

    blackboard->set<geometry_msgs::msg::PoseStamped>("goal_pose", goal_pose);
    RCLCPP_INFO(this->get_logger(),
      "BT goal_pose set to ORIGINAL action goal: (%.3f, %.3f)",
      goal_pose.pose.position.x, goal_pose.pose.position.y);
    
    // Set Task ID (e.g., "navigate_to_pose") to decide branch in BT
    blackboard->set<std::string>("task_id", "navigate_to_pose");

    // 2. Run the loop (Tick)
    RCLCPP_INFO(this->get_logger(), "Executing Static BT for goal...");
    BtStatus status = bt_engine_->run(std::chrono::milliseconds(10)); // 100Hz

    // 3. Finalize
    {
      std::lock_guard<std::mutex> lock(bt_mutex_);
      is_task_running_ = false;

      // 若正在被 handle_restart_task 接管，跳过 action goal 的最终化处理：
      // restart 逻辑会重置此标志并重新启动线程，action goal 生命周期由下一轮 execute_tree 管理。
      if (is_restarting_) {
        RCLCPP_INFO(this->get_logger(), "BT halted for restart, skipping goal finalization.");
        return;
      }

      if (status == BtStatus::SUCCEEDED) {
        RCLCPP_INFO(this->get_logger(), "Navigation task finished successfully.");
        current_goal_handle_->succeed(result);
      } else if (status == BtStatus::CANCELED) {
        if (current_goal_handle_->is_canceling()) {
          RCLCPP_WARN(this->get_logger(), "Navigation task canceled.");
          current_goal_handle_->canceled(result);
        } else {
          RCLCPP_WARN(this->get_logger(), "Navigation task preempted or halted without cancel request.");
          current_goal_handle_->abort(result);
        }
      } else {
        RCLCPP_ERROR(this->get_logger(), "Navigation task failed.");
        current_goal_handle_->abort(result);
      }
      
      // Clear task_id to prevent accidental re-execution if ticked externally
      blackboard->set<std::string>("task_id", "idle");
      has_original_goal_pose_ = false;
    }
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Exception in execute_tree: %s", e.what());
    std::lock_guard<std::mutex> lock(bt_mutex_);
    is_task_running_ = false;
    // is_restarting_ 为 true 时由 restart 逻辑接管，此处不做额外处理
  }
}

} // namespace sp_nav_bt

// ──────────────────────────────────────────────────────────────────────────────
// handle_restart_task
//
// 由 localization_monitor / global_relocalization 在重定位成功后调用。
//
// 执行流程：
//   1. 检查是否有正在运行的任务（无则拒绝）
//   2. 置 is_restarting_=true：通知 execute_tree 结束时不要 abort/succeed goal handle
//   3. halt BT（bt_engine_->halt()）
//   4. 释放锁 → join 旧线程（execute_tree 此时才能拿锁完成退出）
//   5. 重新拿锁 → 清除 is_restarting_，重启 execute_tree 线程
//   6. 新线程强制把黑板 goal_pose 写回 action 受理时缓存的【原始目标】，
//      再重新执行 BT（所有节点从 onStart() 开始）。不得使用地形中间点。
// ──────────────────────────────────────────────────────────────────────────────
namespace sp_nav_bt
{

void NavInterfaceNode::handle_restart_task(
  const std_srvs::srv::Trigger::Request::SharedPtr  /*req*/,
  const std_srvs::srv::Trigger::Response::SharedPtr res)
{
  std::unique_lock<std::mutex> lock(bt_mutex_);

  if (!is_task_running_ || !current_goal_handle_ || !has_original_goal_pose_) {
    res->success = false;
    res->message = "No active navigation task (or original goal) to restart";
    RCLCPP_WARN(this->get_logger(),
      "[restart_task] No active task / original goal, ignoring restart request.");
    return;
  }

  RCLCPP_INFO(this->get_logger(),
    "[restart_task] Relocalization done — halt BT, restart with ORIGINAL goal (%.3f, %.3f)",
    original_goal_pose_.pose.position.x, original_goal_pose_.pose.position.y);

  // 通知 execute_tree：即将重启，结束时不要最终化 goal handle
  is_restarting_ = true;
  bt_engine_->halt();

  // 释放锁，让 execute_tree 能拿锁执行退出逻辑（看到 is_restarting_=true 后直接 return）
  lock.unlock();
  if (bt_thread_.joinable()) {
    bt_thread_.join();
  }
  lock.lock();

  // 清除重启标志，重新启动 execute_tree（execute_tree 内会写回 original_goal_pose_）
  is_restarting_   = false;
  is_task_running_ = true;
  bt_thread_ = std::thread(&NavInterfaceNode::execute_tree, this);

  res->success = true;
  res->message = "BT restarted with original action goal";
  RCLCPP_INFO(this->get_logger(),
    "[restart_task] BT thread restarted with original goal (%.3f, %.3f).",
    original_goal_pose_.pose.position.x, original_goal_pose_.pose.position.y);
}

} // namespace sp_nav_bt

// Main function to run the node
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<sp_nav_bt::NavInterfaceNode>();
  node->init_engine();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
