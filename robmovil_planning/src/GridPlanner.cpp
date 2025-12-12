#include "robmovil_planning/GridPlanner.h"
#include <robmovil_msgs/msg/trajectory.h>
#include <nav_msgs/msg/path.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

robmovil_planning::GridPlanner::GridPlanner() :
    Node("path_planning"), tf_buffer(this->get_clock()), transform_listener_(tf_buffer)
{
  //while( ros::ok() and ros::Time::now() == ros::Time(0) ); // se espera por el primer mensaje de '/clock'
  while((this->now().nanoseconds() == 0));

  rclcpp::QoS qos_profile(rclcpp::KeepLast(10));
  qos_profile.reliable();
  qos_profile.transient_local();

  grid_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>( "/grid", rclcpp::QoS(10), std::bind(&GridPlanner::on_grid, this, std::placeholders::_1));
  goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>( "/planning_goal", rclcpp::QoS(10), std::bind(&GridPlanner::on_goal, this, std::placeholders::_1));

  trajectory_pub_ = this->create_publisher<robmovil_msgs::msg::Trajectory>("/robot/trajectory", qos_profile);
  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/robot/path_planned", rclcpp::QoS(10));
}

void robmovil_planning::GridPlanner::publishPath(const robmovil_msgs::msg::Trajectory& trajectory)
{
  nav_msgs::msg::Path path_msg;
  
  path_msg.header.stamp = trajectory.header.stamp;
  path_msg.header.frame_id = trajectory.header.frame_id;
  
  for(unsigned int i = 0; i < trajectory.points.size(); i++)
  {
    const robmovil_msgs::msg::TrajectoryPoint& wpoint = trajectory.points[i];
    
    geometry_msgs::msg::PoseStamped stamped_pose_msg;
    
    stamped_pose_msg.header.stamp = trajectory.header.stamp;
    stamped_pose_msg.header.frame_id = trajectory.header.frame_id;
    
    stamped_pose_msg.pose.position.x = wpoint.transform.translation.x;
    stamped_pose_msg.pose.position.y = wpoint.transform.translation.y;
    stamped_pose_msg.pose.position.z = wpoint.transform.translation.z;
    stamped_pose_msg.pose.orientation = wpoint.transform.rotation;
    
    path_msg.poses.push_back(stamped_pose_msg);
  }
  
  path_pub_->publish( path_msg );
}

void robmovil_planning::GridPlanner::on_grid(const nav_msgs::msg::OccupancyGrid::SharedPtr grid)
{
  //if (not lookupTransformSafe(transform_listener_, "map", "odom", grid->header.stamp, map_to_odom_))
  //  return;
  try {
  // Try to get the transform at the requested time
  map_to_odom_ = tf_buffer.lookupTransform(
      "map", "odom", grid->header.stamp);
  } catch (const tf2::TransformException &ex) {
    // If that exact time isn't available, fall back to the latest
    try {
      map_to_odom_ = tf_buffer.lookupTransform("map", "odom", tf2::TimePointZero);
    }
    catch (const tf2::TransformException &ex2) {
      RCLCPP_INFO(this->get_logger(), "Could not obtain transform map -> odom");
      return;
    }
  }
  RCLCPP_INFO(this->get_logger(), "New occupancy grid received");

  grid_ = grid;
  
}

void robmovil_planning::GridPlanner::on_goal(const geometry_msgs::msg::PoseStamped::SharedPtr goal)
{
  RCLCPP_INFO(this->get_logger(), "New goal recived");
   
  if(!grid_){ rclcpp::sleep_for(std::chrono::seconds(1)); if(!grid_){ RCLCPP_INFO(this->get_logger(), "No grid message received!"); return;} }
  
  //tf2::poseStampedMsgToTF(goal, goal_pose_);
  goal_pose_ = *goal;
  
  geometry_msgs::msg::TransformStamped odom_to_robot;
  //if (not lookupTransformSafe(transform_listener_, "map", "odom", goal.header.stamp, odom_to_robot))
  //| return;
  try {
  // Try to get the transform at the requested time
  odom_to_robot = tf_buffer.lookupTransform(
      "map", "odom", goal->header.stamp);
  } catch (const tf2::TransformException &ex) {
    // If that exact time isn't available, fall back to the latest
    try {
      odom_to_robot = tf_buffer.lookupTransform("map", "odom", tf2::TimePointZero);
    }
    catch (const tf2::TransformException &ex2) {
      RCLCPP_INFO(this->get_logger(), "Could not obtain transform map -> odom");
      return;
    }
  }
    
  starting_pose_.header.stamp = odom_to_robot.header.stamp;
  starting_pose_.header.frame_id = odom_to_robot.header.frame_id;
  tf2::Transform tf_start(
        tf2::Quaternion(
            odom_to_robot.transform.rotation.x,
            odom_to_robot.transform.rotation.y,
            odom_to_robot.transform.rotation.z,
            odom_to_robot.transform.rotation.w),
        tf2::Vector3(
            odom_to_robot.transform.translation.x,
            odom_to_robot.transform.translation.y,
            odom_to_robot.transform.translation.z)
  );

  tf2::toMsg(tf_start, starting_pose_.pose);

  robmovil_msgs::msg::Trajectory trajectory_msg;
  
  if(do_planning(trajectory_msg)){
    RCLCPP_INFO(this->get_logger(), "A plan has been found");
    
    trajectory_msg.header.stamp = this->now();
    trajectory_msg.header.frame_id = "odom";
    
    trajectory_pub_->publish( trajectory_msg );
    publishPath(trajectory_msg);
  }else
    RCLCPP_INFO(this->get_logger(), "No plan has been found");
}

bool robmovil_planning::GridPlanner::getOriginOfCell(uint i, uint j, double& x, double& y)
{
  if(!grid_ or i < 0 or j < 0 or i >= grid_->info.width or j >= grid_->info.height)
    return false;
  
  x = grid_->info.origin.position.x + i * grid_->info.resolution;
  y = grid_->info.origin.position.y + j * grid_->info.resolution;
  
  return true;
}

bool robmovil_planning::GridPlanner::getCenterOfCell(uint i, uint j, double& x, double& y)
{
  if (!getOriginOfCell(i,j,x,y))
    return false;
    
  x += grid_->info.resolution / 2;
  y += grid_->info.resolution / 2;
  
  return true;
}

inline double rounddown_nearest(const double& num, const double& multiple)
{
  double r = std::fmod(num, multiple);
  return num < 0 ? num + r : num - r;
}

bool robmovil_planning::GridPlanner::getCellOfPosition(const double& x, const double& y, uint& i, uint& j)
{
  if(!grid_)
    return false;
  
  double floor_width = grid_->info.width * grid_->info.resolution;
  double floor_height = grid_->info.height * grid_->info.resolution;
  
  double tmp_i = (rounddown_nearest(x, grid_->info.resolution) + floor_width/2) / grid_->info.resolution;
  double tmp_j = (rounddown_nearest(y, grid_->info.resolution) + floor_height/2) / grid_->info.resolution;
  
  if(tmp_i < 0 or tmp_j < 0 or tmp_i >= grid_->info.width or tmp_j >= grid_->info.height)
    return false;
    
  i = tmp_i;
  j = tmp_j;
  
  return true;
}

bool robmovil_planning::GridPlanner::isCellOccupy(uint i, uint j)
{
  if(!grid_ or i < 0 or j < 0 or i >= grid_->info.width or j >= grid_->info.height)
    return false;
  
  return grid_->data[i + grid_->info.width * j];
}

bool robmovil_planning::GridPlanner::isANeighborCellOccupy(uint i, uint j)
{
  if(!grid_ or i < 0 or j < 0 or i >= grid_->info.width or j >= grid_->info.height)
    return false;
    
  return isCellOccupy(i-1,j-1) or isCellOccupy(i-1,j) or isCellOccupy(i-1,j+1) or isCellOccupy(i,j-1) or 
         isCellOccupy(i,j+1) or isCellOccupy(i+1,j-1) or isCellOccupy(i+1,j) or isCellOccupy(i+1,j+1);
}

bool robmovil_planning::GridPlanner::isPositionOccupy(const double& x, const double& y)
{
  uint i, j;
  
  if(!getCellOfPosition(x,y,i,j))
    return false;
  
  return isCellOccupy(i,j);
}

bool robmovil_planning::GridPlanner::do_planning(robmovil_msgs::msg::Trajectory& result_trajectory)
{
  //ROS_INFO_STREAM("do_planning: " << " Test implementation for debugging!");
  RCLCPP_INFO(this->get_logger(), "do_planning: Test implementation for debugging!");
  
  RCLCPP_INFO(this->get_logger(), "Grid: resolution: %.2f, width: %u, height: %u, origin (%.2f, %.2f, %.2f)", grid_->info.resolution, grid_->info.width, grid_->info.height, grid_->info.origin.position.x, grid_->info.origin.position.y, grid_->info.origin.position.z);
    
  uint i, j;
  
  double x = starting_pose_.pose.position.x;
  double y = starting_pose_.pose.position.y;
  
  getCellOfPosition(x, y, i, j);
  
  RCLCPP_INFO(this->get_logger(), "Starting_pose: %.2f, %.2f; Cell: %u, %u", x, y, i, j);

  x = goal_pose_.pose.position.x;
  y = goal_pose_.pose.position.y;
  
  getCellOfPosition(x, y, i, j);
  
  RCLCPP_INFO(this->get_logger(), "Goal_pose: %.2f, %.2f; Cell: %u, %u", x, y, i, j);
  
  x = -4.71;
  y = 4.17;
  
  getCellOfPosition(x, y, i, j);
  
  RCLCPP_INFO(this->get_logger(), "Pos: %.2f, %.2f; Cell: %u, %u", x, y, i, j);
  
  
  x = -0.5;
  y = -0.5;
  
  getCellOfPosition(x, y, i, j);
  
  RCLCPP_INFO(this->get_logger(), "Pos: %.2f, %.2f; Cell: %u, %u", x, y, i, j);
  
  x = -0.25;
  y = -0.25;
  
  getCellOfPosition(x, y, i, j);
  
  RCLCPP_INFO(this->get_logger(), "Pos: %.2f, %.2f; Cell: %u, %u", x, y, i, j);
  
  x = 0.25;
  y = 0.25;
  
  getCellOfPosition(x, y, i, j);
  
  RCLCPP_INFO(this->get_logger(), "Pos: %.2f, %.2f; Cell: %u, %u", x, y, i, j);
  
  x = -7.5;
  y = -7.5;
  
  RCLCPP_INFO(this->get_logger(), "Pos: %.2f, %.2f; Cell: %u, %u; Occupy: %u", x, y, i, j, isPositionOccupy(x,y));
  return false;
}
