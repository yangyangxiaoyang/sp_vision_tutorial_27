#include <rclcpp/rclcpp.hpp>

#include "sp_controller_server/controller_server.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<sp_controller_server::ControllerServer>();
  node->init();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
