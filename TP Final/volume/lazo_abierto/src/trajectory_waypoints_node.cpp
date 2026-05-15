#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <vector>

struct waypoint {
  double time_from_start;
  double target_lin_vel;
  double target_ang_vel;
};

std::vector<waypoint> planBox()
{
  std::vector<waypoint> waypoints;
  
  waypoints.push_back({0, 0.25, 0});
  waypoints.push_back({8, 0.25, 0});
  waypoints.push_back({8, 0.25, -M_PI/4});
  waypoints.push_back({10, 0.25, 0});
  waypoints.push_back({15, 0, 0});
  
  return waypoints;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::QoS qos_profile(rclcpp::KeepLast(1));
  qos_profile.reliable();
  qos_profile.transient_local();

  auto trajectory_waypoints_node = rclcpp::Node::make_shared("trajectory_waypoints");
  
  trajectory_publisher = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/robot/trajectory", qos_profile);

  // Path descripto en poses para visualizacion en RViz

  trajectory_msgs::msg::JointTrajectory trajectory_msg;
  nav_msgs::msg::Path path_msg;
  geometry_msgs::msg::PoseArray poses_msg;

  //trajectory_msg.header.seq = 0;
  trajectory_msg.header.stamp = trajectory_waypoints_node->now();
  trajectory_msg.header.frame_id = "odom";

  trajectory_msg.joint_names.push_back("x");
  trajectory_msg.joint_names.push_back("y");
  trajectory_msg.joint_names.push_back("theta");
  
  std::vector<waypoint> waypoints = planBox();

  for (waypoint wpoint : waypoints)
  {
    double t = wpoint.time_from_start;
    double vl = wpoint.target_lin_vel;
    double va = wpoint.target_ang_vel;

    trajectory_msgs::msg::JointTrajectoryPoint point_msg;
    
    point_msg.time_from_start = rclcpp::Duration::from_seconds(t);

    point_msg.velocities.push_back( vl );
    point_msg.velocities.push_back( va );

    trajectory_msg.points.push_back( point_msg );
  }

  //trajectory_publisher->publish( trajectory_msg );

  rclcpp::spin(trajectory_waypoints_node);
  rclcpp::shutdown();

  return 0;
}
