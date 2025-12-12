#include <rclcpp/rclcpp.hpp>
#include <lazo_abierto/FeedForwardController.h>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto trajectory_follower = std::make_shared<FeedForwardController>();
  rclcpp::spin(trajectory_follower);
  
  trajectory_follower->stop_timer();
  rclcpp::shutdown();

  return 0;
}
