#include "sp_global_planner/planner_server.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<sp_global_planner::PlannerServer>();
  node->init(); 
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
