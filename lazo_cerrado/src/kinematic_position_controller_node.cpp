#include <rclcpp/rclcpp.hpp>
#include "KinematicPositionController.h"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto kinematic_position_controller_node = std::make_shared<KinematicPositionController>();
  rclcpp::spin(kinematic_position_controller_node);
  kinematic_position_controller_node->stop_timer();
  rclcpp::shutdown();
  return 0;
}
