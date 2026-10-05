#include <rclcpp/rclcpp.hpp>
#include "pioneer_odometry.h"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  // NUEVO para visualizar base_link
//  rclcpp::NodeOptions options;
//  options.append_parameter_override("use_sim_time", true);
  //
  rclcpp::spin((std::make_shared<robmovil::PioneerOdometry>()));
  rclcpp::shutdown();
  return 0;
}
