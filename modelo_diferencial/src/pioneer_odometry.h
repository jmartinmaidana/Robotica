#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <robmovil_msgs/msg/encoder_ticks.hpp>
#include <std_msgs/msg/float64.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>

namespace robmovil
{

class PioneerOdometry : public rclcpp::Node
{
  public:

    PioneerOdometry();

    void on_velocity_cmd(const geometry_msgs::msg::Twist::SharedPtr twist);

    void on_encoder_ticks(const robmovil_msgs::msg::EncoderTicks::SharedPtr encoder);

  private:

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
    rclcpp::Subscription<robmovil_msgs::msg::EncoderTicks>::SharedPtr encoder_sub_;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr vel_pub_left_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr vel_pub_right_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odometry_;

  // Acá pueden agregar las variables de instancia que necesiten
  // ...

    double x_, y_, theta_;

    bool ticks_initialized_;
    int32_t last_ticks_left_, last_ticks_right_;
    rclcpp::Time last_ticks_time;

    //boost::shared_ptr<tf::TransformBroadcaster> tf_broadcaster;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

}
