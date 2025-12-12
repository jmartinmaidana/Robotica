#include <rclcpp/rclcpp.hpp>
#include "PioneerRRTPlanner.h"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto path_planning = std::make_shared<robmovil_planning::PioneerRRTPlanner>();
  rclcpp::spin(path_planning);
  rclcpp::shutdown();
  return 0;
}
