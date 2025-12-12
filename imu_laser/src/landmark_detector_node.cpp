#include <rclcpp/rclcpp.hpp>
#include "landmark_detector.h"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto landmark_detector_node = std::make_shared<robmovil_ekf::LandmarkDetector>();
  rclcpp::spin(landmark_detector_node);
  rclcpp::shutdown();
  return 0;
}

