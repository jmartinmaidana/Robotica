#include <lazo_abierto/TrajectoryFollower.h>
#include <geometry_msgs/msg/twist.hpp>

TrajectoryFollower::TrajectoryFollower() : Node("nodeTrajectoryFollower")
{
  rclcpp::QoS qos_profile(rclcpp::KeepLast(10));
  qos_profile.reliable();
  qos_profile.transient_local();

  cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", rclcpp::QoS(10));

  trajectory_sub_ = this->create_subscription<robmovil_msgs::msg::Trajectory>("/robot/trajectory", qos_profile, std::bind(&TrajectoryFollower::handleNewTrajectory, this, std::placeholders::_1));
}

void TrajectoryFollower::handleNewTrajectory(const robmovil_msgs::msg::Trajectory& trajectory_msg)
{
  RCLCPP_INFO(this->get_logger(), "New trajectory recived");

  do { t0_ = this->now(); } while (t0_.seconds() == 0.0);
  
  RCLCPP_INFO(this->get_logger(), "t0: %.9f", t0_.seconds());
  RCLCPP_INFO(this->get_logger(), "trajectory size: %zu", trajectory_msg.points.size());

  current_trajectory_ = trajectory_msg;

  timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&TrajectoryFollower::timerCallback, this));
}

void TrajectoryFollower::timerCallback()
{
  rclcpp::Time t = this->now();
  RCLCPP_INFO(this->get_logger(), "t: %.9f", t.seconds());

  // Aplicar la ley de control

  double v, w;
  if( not control(t, v, w) ) {
    RCLCPP_INFO(this->get_logger(), "Trajectory finished");
    timer_->cancel();
    
    geometry_msgs::msg::Twist cmd;
    cmd_pub_->publish( cmd ); // se detiene luego de terminar el trayecto
    return;
  }

  // Crear mensaje

  geometry_msgs::msg::Twist cmd;

  cmd.linear.x = v;
  cmd.linear.y = 0;
  cmd.linear.z = 0;

  cmd.angular.x = 0;
  cmd.angular.y = 0;
  cmd.angular.z = w;

  cmd_pub_->publish( cmd );
}

bool TrajectoryFollower::nextPointIndex(const rclcpp::Time& time, size_t& next_point_idx) const
{
  size_t idx = 0;
  for ( const robmovil_msgs::msg::TrajectoryPoint& point : current_trajectory_.points )
  {
    if ( time < t0_ + point.time_from_start )
    {
      next_point_idx = idx;
      return true;
    }
    idx++;
  }

  return false;
}
