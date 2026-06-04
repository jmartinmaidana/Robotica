#ifndef __ROBMOVIL_EKF_LANDMARK_DETECTOR_H__
#define __ROBMOVIL_EKF_LANDMARK_DETECTOR_H__

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <robmovil_msgs/msg/landmark_array.hpp>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

namespace robmovil_ekf
{
  /*
   * Calcula landmarks representados en coordenadas polares, en el sistema de referencia del robot
   */

  class LandmarkDetector : public rclcpp::Node
  {
    public:
      LandmarkDetector();

      void on_laser_scan(const sensor_msgs::msg::LaserScan::SharedPtr msg);

    private:
      void publish_pointcloud(const std_msgs::msg::Header& header, const std::vector<tf2::Vector3>& landmark_positions);
      bool update_laser_tf(const rclcpp::Time& required_time);

      rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub;
      rclcpp::Publisher<robmovil_msgs::msg::LandmarkArray>::SharedPtr landmark_pub;
      rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr pointcloud_pub;

      std::string robot_frame, laser_frame, publish_robot_frame;

      tf2_ros::Buffer tf_buffer;
      tf2_ros::TransformListener tf_listener;
      tf2::Transform laser_transform;
      bool transform_received;
  };
}

#endif // __ROBMOVIL_EKF_LANDMARK_DETECTOR_H__
