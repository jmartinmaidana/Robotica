#include <rclcpp/rclcpp.hpp>
#include "robmovil_planning/GridPlanner.h"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto path_planning = std::make_shared<robmovil_planning::GridPlanner>();
  rclcpp::spin(path_planning);
  rclcpp::shutdown();
  return 0;
}
