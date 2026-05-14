#include <rclcpp/rclcpp.hpp>
#include "pioneer_odometry.h"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin((std::make_shared<robmovil::PioneerOdometry>()));
  rclcpp::shutdown();
  return 0;
}
