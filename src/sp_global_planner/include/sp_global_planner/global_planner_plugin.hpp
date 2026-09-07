#pragma once
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

namespace sp_global_planner {

class GlobalPlannerPlugin
{
public:
  virtual ~GlobalPlannerPlugin() = default;

  // Called once after plugin creation
  virtual void configure(
    const rclcpp::Node::SharedPtr& node,
    const std::string& plugin_name) = 0;

  // Called whenever map updates
  virtual void setMap(const nav_msgs::msg::OccupancyGrid& costmap) = 0;

  // Create a plan from start to goal (both in map frame)
  virtual nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped& start,
    const geometry_msgs::msg::PoseStamped& goal) = 0;
};

} // namespace sp_global_planner
