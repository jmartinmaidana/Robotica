#include <rclcpp/rclcpp.hpp>
#include "keys_to_twist.h"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<robmovil::KeysToTwist>());
  rclcpp::shutdown();
  return 0;
}

