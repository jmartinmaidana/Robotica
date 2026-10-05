#ifndef __ROBMOVIL_EKF_LOCALIZER_H__
#define __ROBMOVIL_EKF_LOCALIZER_H__

#include <rclcpp/rclcpp.hpp>
#include <robmovil_msgs/msg/landmark_array.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Transform.h>

#include "localizer_ekf.h"

namespace robmovil_ekf {
  class Localizer : public rclcpp::Node
  {
    public:
      Localizer();

      void on_landmark_array(const robmovil_msgs::msg::LandmarkArray::SharedPtr msg);
      void on_imu(const sensor_msgs::msg::Imu::SharedPtr msg);
      void on_odometry(const nav_msgs::msg::Odometry::SharedPtr msg);

    private:
      LocalizerEKF ekf;
      bool set_map;
      bool only_prediction;
      int min_landmark_size;
      
      rclcpp::Subscription<robmovil_msgs::msg::LandmarkArray>::SharedPtr landmark_sub;
      rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
      rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odo_sub;
      rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub;
      std::string base_frame_, map_frame_, laser_frame_;
      
      rclcpp::TimerBase::SharedPtr prediction_timer;
      std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
      
      void prediction_event();
      void advance_time(const rclcpp::Time& now);
      void publish_estimate(const rclcpp::Time& now);
      
      void prediction(const rclcpp::Time& time);
      
      /* ultimo comando de control recibido */
      LocalizerEKF::Vector u;
      rclcpp::Time t;
  };
}

#endif // __ROBMOVIL_EKF_LOCALIZER_H__
