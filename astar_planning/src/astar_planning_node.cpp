#include <rclcpp/rclcpp.hpp>
#include "AStarPlanner.h"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto path_planning = std::make_shared<robmovil_planning::AStarPlanner>();
  rclcpp::spin(path_planning);
  rclcpp::shutdown();
  return 0;
}
