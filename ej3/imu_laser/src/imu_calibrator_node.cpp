#include <rclcpp/rclcpp.hpp>
#include "imu_calibrator.h"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto imu_calibrator_node = std::make_shared<robmovil_ekf::IMUCalibrator>();
  rclcpp::spin(imu_calibrator_node);
  rclcpp::shutdown();
  return 0;
}
