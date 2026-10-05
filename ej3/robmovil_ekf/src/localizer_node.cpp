#include <rclcpp/rclcpp.hpp>
#include "localizer.h"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto localizer_node = std::make_shared<robmovil_ekf::Localizer>();
  rclcpp::spin(localizer_node);
  rclcpp::shutdown();
  return 0;
}
