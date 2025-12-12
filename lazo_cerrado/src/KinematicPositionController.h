#pragma once

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/utils.h>
#include <lazo_abierto/TrajectoryFollower.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "nav_msgs/msg/odometry.hpp"

class KinematicPositionController : public TrajectoryFollower
{
  public:
    
    enum GoalSelectionType { TIME_BASED, PURSUIT_BASED, FIXED_GOAL };
    
    KinematicPositionController();

    bool control(const rclcpp::Time& t, double& v, double& w);

  private:

    tf2_ros::Buffer tfBuffer_;
    tf2_ros::TransformListener transform_listener_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr expected_position_pub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr current_pos_sub_;

    
    GoalSelectionType goal_selection_;
    
    double fixed_goal_x_;
    double fixed_goal_y_;
    double fixed_goal_a_;
    
    double x = 0.0;
    double y = 0.0;
    double a = 0.0;
    
    // funciones auxiliares
    
    void getCurrentPoseFromOdometry(const nav_msgs::msg::Odometry & msg);
    
    bool getCurrentGoal(const rclcpp::Time& t, double& x, double& y, double& a)
    {
      switch(goal_selection_)
      {
        case TIME_BASED: return getTimeBasedGoal(t, x, y, a);
        case PURSUIT_BASED: return getPursuitBasedGoal(t, x, y, a);
        case FIXED_GOAL: x = fixed_goal_x_; y = fixed_goal_y_; a = fixed_goal_a_; return true;
        default: return getTimeBasedGoal(t, x, y, a);
      }
    }
    
    bool getTimeBasedGoal(const rclcpp::Time& t, double& x, double& y, double& a);
    bool getPursuitBasedGoal(const rclcpp::Time& t, double& x, double& y, double& a);
    
    void publishCurrentGoal(const rclcpp::Time& t, const double& goal_x, const double& goal_y, const double& goal_a)
    {
      geometry_msgs::msg::PoseStamped expected_pose;
      expected_pose.header.frame_id = "map";
      expected_pose.header.stamp = t;
      expected_pose.pose.position.x = goal_x;
      expected_pose.pose.position.y = goal_y;
      expected_pose.pose.position.z = 0;
      tf2::Quaternion orientation;
      orientation.setRPY(0,0,goal_a);
      expected_pose.pose.orientation = tf2::toMsg(orientation);
      expected_position_pub->publish(expected_pose);
    }
};
