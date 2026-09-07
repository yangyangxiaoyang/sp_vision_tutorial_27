#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robot_msg/action/navigate_to_pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/string.hpp"

namespace sp_nav_bt
{

class NavActionClient : public rclcpp::Node
{
public:
  using NavigateToPose = robot_msg::action::NavigateToPose;
  using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  explicit NavActionClient(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("nav_action_client", options)
  {
    this->client_ptr_ = rclcpp_action::create_client<NavigateToPose>(
      this,
      "navigate_to_pose");

    this->pose_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "goal_pose", 10, std::bind(&NavActionClient::pose_callback, this, std::placeholders::_1));

    this->cmd_subscription_ = this->create_subscription<std_msgs::msg::String>(
      "nav_cmd", 10, std::bind(&NavActionClient::cmd_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "NavActionClient initialized.");
    RCLCPP_INFO(this->get_logger(), "Subscribe to 'goal_pose' for new goals.");
    RCLCPP_INFO(this->get_logger(), "Subscribe to 'nav_cmd' with 'cancel' to stop current task.");
  }

  void send_goal(const geometry_msgs::msg::PoseStamped & pose)
  {
    using namespace std::placeholders;

    if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(5))) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      return;
    }

    // Cancel existing goal if any (simplified task switching)
    if (this->goal_handle_ && 
        (this->goal_handle_->get_status() == action_msgs::msg::GoalStatus::STATUS_ACCEPTED ||
         this->goal_handle_->get_status() == action_msgs::msg::GoalStatus::STATUS_EXECUTING)) {
      RCLCPP_INFO(this->get_logger(), "Canceling previous goal before sending new one");
      this->client_ptr_->async_cancel_goal(this->goal_handle_);
    }

    auto goal_msg = NavigateToPose::Goal();
    goal_msg.pose = pose;
    goal_msg.behavior_tree = ""; // Default BT

    RCLCPP_INFO(this->get_logger(), "Sending goal to: (%.2f, %.2f)", 
      pose.pose.position.x, pose.pose.position.y);

    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      std::bind(&NavActionClient::goal_response_callback, this, _1);
    send_goal_options.feedback_callback =
      std::bind(&NavActionClient::feedback_callback, this, _1, _2);
    send_goal_options.result_callback =
      std::bind(&NavActionClient::result_callback, this, _1);
    
    this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

  void cancel_goal()
  {
    if (this->goal_handle_ && 
        (this->goal_handle_->get_status() == action_msgs::msg::GoalStatus::STATUS_ACCEPTED ||
         this->goal_handle_->get_status() == action_msgs::msg::GoalStatus::STATUS_EXECUTING)) {
      RCLCPP_INFO(this->get_logger(), "Canceling current goal...");
      this->client_ptr_->async_cancel_goal(this->goal_handle_);
    } else {
      RCLCPP_WARN(this->get_logger(), "No active goal to cancel.");
    }
  }

private:
  rclcpp_action::Client<NavigateToPose>::SharedPtr client_ptr_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr cmd_subscription_;
  GoalHandleNav::SharedPtr goal_handle_;

  void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    this->send_goal(*msg);
  }

  void cmd_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    if (msg->data == "cancel") {
      this->cancel_goal();
    } else {
      RCLCPP_WARN(this->get_logger(), "Unknown command: %s", msg->data.c_str());
    }
  }

  void goal_response_callback(const GoalHandleNav::SharedPtr & goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      this->goal_handle_ = goal_handle;
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
  }

  void feedback_callback(
    GoalHandleNav::SharedPtr,
    const std::shared_ptr<const NavigateToPose::Feedback> feedback)
  {
    RCLCPP_INFO(this->get_logger(), "Received feedback: distance_remaining = %.2f", 
      feedback->distance_remaining);
  }

  void result_callback(const GoalHandleNav::WrappedResult & result)
  {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(this->get_logger(), "Goal succeeded!");
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
        break;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_INFO(this->get_logger(), "Goal was canceled");
        break;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        break;
    }
    this->goal_handle_.reset();
  }
};

} // namespace sp_nav_bt

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<sp_nav_bt::NavActionClient>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
