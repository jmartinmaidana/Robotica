#include "pioneer_odometry.h"
#include <std_msgs/msg/float64.hpp>
#include <nav_msgs/msg/odometry.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>
using namespace robmovil;

// NUEVO
// NUEVO
// NUEVO
// NUEVO
// NUEVO
// NUEVO

#define L_X = 0.175
#define L_Y = 0.175
#define WHEEL_RADIUS 0.050
#define ENCODER_TICKS 500.0

PioneerOdometry::PioneerOdometry() : Node("nodeOdometry"), x_(0), y_(0), theta_(0), ticks_initialized_(false)
{
  // Nos suscribimos a los comandos de velocidad en el tópico "/robot/cmd_vel" de tipo geometry_msgs::Twist
  twist_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("/robot/cmd_vel", rclcpp::QoS(10), std::bind(&PioneerOdometry::on_velocity_cmd, this, std::placeholders::_1));

  vel_pub_front_left_  = this->create_publisher<std_msgs::msg::Float64>("/robot/front_left_wheel/cmd_vel", rclcpp::QoS(10));
  vel_pub_front_right_ = this->create_publisher<std_msgs::msg::Float64>("/robot/front_right_wheel/cmd_vel", rclcpp::QoS(10));
  vel_pub_rear_left_   = this->create_publisher<std_msgs::msg::Float64>("/robot/rear_left_wheel/cmd_vel", rclcpp::QoS(10));
  vel_pub_rear_right_  = this->create_publisher<std_msgs::msg::Float64>("/robot/rear_right_wheel/cmd_vel", rclcpp::QoS(10));
  

  encoder_sub_ =  this->create_subscription<robmovil_msgs::msg::MultiEncoderTicks>(
          "/robot/encoders", rclcpp::QoS(10), std::bind(&PioneerOdometry::on_encoder_ticks, this, std::placeholders::_1));
  
  pub_odometry_ = this->create_publisher<nav_msgs::msg::Odometry>("/robot/odometry", rclcpp::QoS(10));
  
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
}

void PioneerOdometry::on_velocity_cmd(const geometry_msgs::msg::Twist::SharedPtr twist)
{
  double linear_x_velocity  = twist->linear.x;   //longitudinal velocity
  double linear_y_velocity  = twist->linear.y;   //transversal velocity, no se utiliza en este robot
  double angular_velocisgty = twist->angular.z;

  double vFront_left   = (linear_x_velocity - linear_y_velocity - (L_X + L_Y) * angular_velocity) / WHEEL_RADIUS;
  double vFront_right  = (linear_x_velocity + linear_y_velocity + (L_X + L_Y) * angular_velocity) / WHEEL_RADIUS;
  double vRear_left    = (linear_x_velocity + linear_y_velocity - (L_X + L_Y) * angular_velocity) / WHEEL_RADIUS;
  double vRear_right   = (linear_x_velocity - linear_y_velocity + (L_X + L_Y) * angular_velocity) / WHEEL_RADIUS;

  // publish front_left velocity
  {
    std_msgs::msg::Float64 msg;
    msg.data = vLeft;

    vel_pub_front_left_->publish(msg);
  }

  // publish front_right velocity
  {
    std_msgs::msg::Float64 msg;
    msg.data = vRight;

    vel_pub_front_right_->publish(msg);
  }

  // publish rear_left velocity
  {
    std_msgs::msg::Float64 msg;
    msg.data = vRight;

    vel_pub_rear_left_->publish(msg);
  }

  // publish rear_right velocity
  {
    last_ticks_front_left_ = encoder->ticks_left.data;
    last_ticks_front_right_ = encoder->ticks_right.data;
    last_ticks_rear_left_ = encoder->ticks_right.data;
    std_msgs::msg::Float64 msg;
    msg.data = vRight;

    vel_pub_rear_right_->publish(msg);
  }
}


void PioneerOdometry::on_encoder_ticks(const robmovil_msgs::msg::MultiEncoderTicks::SharedPtr encoder)
{
  // La primera vez que llega un mensaje de encoders
  // inicializo las variables de estado.
  if (!ticks_initialized_) {
    ticks_initialized_ = true;
    last_ticks_front_left_  = encoder->ticks[0].data;
    last_ticks_front_right_ = encoder->ticks[1].data;
    last_ticks_rear_left_   = encoder->ticks[2].data;
    last_ticks_rear_right_  = encoder->ticks[3].data;
    last_ticks_time = encoder->header.stamp;
    return;
  }

  int32_t delta_ticks_front_left  = encoder->ticks[0].data - last_ticks_front_left_;
  int32_t delta_ticks_front_right = encoder->ticks[1].data - last_ticks_front_right_;
  int32_t delta_ticks_rear_left   = encoder->ticks[2].data - last_ticks_rear_left_;
  int32_t delta_ticks_rear_right  = encoder->ticks[3].data - last_ticks_rear_right_;


  /* Utilizar este delta de tiempo entre momentos */
  rclcpp::Time current_time(encoder->header.stamp);
  double delta_t = (current_time - last_ticks_time).seconds();

  double d_front_left  = (delta_ticks_front_left  * WHEEL_RADIUS * 2.0 * M_PI) / ENCODER_TICKS;
  double d_front_right = (delta_ticks_front_left  * WHEEL_RADIUS * 2.0 * M_PI) / ENCODER_TICKS;
  double d_rear_left   = (delta_ticks_rear_left   * WHEEL_RADIUS * 2.0 * M_PI) / ENCODER_TICKS;
  double d_rear_right  = (delta_ticks_rear_right  * WHEEL_RADIUS * 2.0 * M_PI) / ENCODER_TICKS;
  double d = (d_front_left + d_front_right + d_rear_left + d_rear_right) / 4.0;

  // calcular el desplazamiento relativo: delta_x, delta_y, delta_theta
  double delta_theta = (d_front_right + d_rear_right - d_front_left - d_rear_left) / (L_X + L_Y);
  double delta_x = d * cos(theta_);
  double delta_y = d * sin(theta_);
  //double delta_x = d * cos(theta_ + delta_theta / 2.0);
  //double delta_y = d * sin(theta_ + delta_theta / 2.0);
  
  
  /*--------------------
  // Cinemática Directa en el marco del robot (Ecuaciones 22, 23 y 24)
  double delta_x_local = (d_fl + d_fr + d_rl + d_rr) / 4.0;
  double delta_y_local = (-d_fl + d_fr + d_rl - d_rr) / 4.0;
  double delta_theta   = (-d_fl + d_fr - d_rl + d_rr) / (4.0 * (L_X + L_Y));

  // Transformación al marco global del mapa (Odometría)
  x_ += delta_x_local * cos(theta_) - delta_y_local * sin(theta_);
  y_ += delta_x_local * sin(theta_) + delta_y_local * cos(theta_);
  theta_ += delta_theta;
  ----------------*/



  /** Utilizar variables globales x_, y_, theta_ definidas en el .h */
  //Calculo la nueva pose
  //x_ += d * cos(theta_);
  //y_ += d * sin(theta_);
  x_ += delta_x;
  y_ += delta_y;
  theta_ += delta_theta;



//VISUALIZACIÓN EN CONSOLA

  double normalized_theta_rad = fmod(theta_, 2.0 * M_PI);
  if (normalized_theta_rad < 0) {
    normalized_theta_rad += 2.0 * M_PI;
  }
  // 2. Convertir a grados
  double theta_en_grados = normalized_theta_rad * 180.0 / M_PI;

  //calculo velocidades
  double linear_velocity = d / delta_t;
  double angular_velocity = delta_theta / delta_t;

  RCLCPP_INFO(this->get_logger(), 
              "Pose -> X: %.2f m, Y: %.2f m, Theta: %.2f deg | Vel -> Lin: %.2f m/s, Ang: %.2f rad/s", 
              x_, y_, theta_en_grados, linear_velocity, angular_velocity);// Construir el mensaje odometry utilizando el esqueleto siguiente:
  nav_msgs::msg::Odometry msg;

//END VISUALIZACIÓN EN CONSOLA


  msg.header.stamp = encoder->header.stamp;
  msg.header.frame_id = "odom";
  msg.child_frame_id = "base_link";

  msg.pose.pose.position.x = x_;
  msg.pose.pose.position.y = y_;
  msg.pose.pose.position.z = 0;

  tf2::Quaternion q;
  q.setRPY(0, 0, theta_);  // roll, pitch, yaw
  msg.pose.pose.orientation = tf2::toMsg(q);

  msg.twist.twist.linear.x = d / delta_t;
  msg.twist.twist.linear.y = 0;
  msg.twist.twist.linear.z = 0;

  msg.twist.twist.angular.x = 0;
  msg.twist.twist.angular.y = 0;
  msg.twist.twist.angular.z = delta_theta / delta_t;

  pub_odometry_->publish( msg );

  // Actualizo las variables de estado

  last_ticks_left_ = encoder->ticks_left.data;
  last_ticks_right_ = encoder->ticks_right.data;
  last_ticks_time = current_time;

  /* Mando tambien un transform usando TF */

  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = this->get_clock()->now();
  t.header.frame_id = "odom";
  t.child_frame_id = "base_link";
  t.transform.translation.x = msg.pose.pose.position.x;
  t.transform.translation.y = msg.pose.pose.position.y;
  t.transform.translation.z = msg.pose.pose.position.z;
  t.transform.rotation = msg.pose.pose.orientation;

  tf_broadcaster_->sendTransform(t);


}


/*

void OmniOdometry::on_encoder_ticks(const robmovil_msgs::msg::MultiEncoderTicks::SharedPtr encoder)
{
  if (!ticks_initialized_) {
    ticks_initialized_ = true;
    last_ticks_fl_ = encoder->ticks[0].data;
    last_ticks_fr_ = encoder->ticks[1].data;
    last_ticks_rl_ = encoder->ticks[2].data;
    last_ticks_rr_ = encoder->ticks[3].data;
    last_ticks_time = encoder->header.stamp;
    return;
  }

  // Diferencia de ticks
  int32_t delta_ticks_fl = encoder->ticks[0].data - last_ticks_fl_;
  int32_t delta_ticks_fr = encoder->ticks[1].data - last_ticks_fr_;
  int32_t delta_ticks_rl = encoder->ticks[2].data - last_ticks_rl_;
  int32_t delta_ticks_rr = encoder->ticks[3].data - last_ticks_rr_;

  rclcpp::Time current_time(encoder->header.stamp);
  double delta_t = (current_time - last_ticks_time).seconds();

  // Distancia recorrida por cada rueda
  double d_fl = (delta_ticks_fl * 2.0 * M_PI * WHEEL_RADIUS) / ENCODER_TICKS;
  double d_fr = (delta_ticks_fr * 2.0 * M_PI * WHEEL_RADIUS) / ENCODER_TICKS;
  double d_rl = (delta_ticks_rl * 2.0 * M_PI * WHEEL_RADIUS) / ENCODER_TICKS;
  double d_rr = (delta_ticks_rr * 2.0 * M_PI * WHEEL_RADIUS) / ENCODER_TICKS;

  // Cinemática Directa en el marco del robot (Ecuaciones 22, 23 y 24)
  double delta_x_local = (d_fl + d_fr + d_rl + d_rr) / 4.0;
  double delta_y_local = (-d_fl + d_fr + d_rl - d_rr) / 4.0;
  double delta_theta   = (-d_fl + d_fr - d_rl + d_rr) / (4.0 * (L_X + L_Y));

  // Transformación al marco global del mapa (Odometría)
  x_ += delta_x_local * cos(theta_) - delta_y_local * sin(theta_);
  y_ += delta_x_local * sin(theta_) + delta_y_local * cos(theta_);
  theta_ += delta_theta;

  // Visualización en consola
  double theta_en_grados = fmod(theta_, 2.0 * M_PI) * 180.0 / M_PI;
  if (theta_en_grados < 0) theta_en_grados += 360.0;

  RCLCPP_INFO(this->get_logger(), 
              "Pose -> X: %.2f m, Y: %.2f m, Theta: %.2f deg", 
              x_, y_, theta_en_grados);

  // Armado del mensaje Odometry
  nav_msgs::msg::Odometry msg;
  msg.header.stamp = encoder->header.stamp;
  msg.header.frame_id = "odom";
  msg.child_frame_id = "base_link";

  msg.pose.pose.position.x = x_;
  msg.pose.pose.position.y = y_;
  msg.pose.pose.position.z = 0;

  tf2::Quaternion q;
  q.setRPY(0, 0, theta_);
  msg.pose.pose.orientation = tf2::toMsg(q);

  msg.twist.twist.linear.x = delta_x_local / delta_t;
  msg.twist.twist.linear.y = delta_y_local / delta_t;
  msg.twist.twist.linear.z = 0;
  msg.twist.twist.angular.x = 0;
  msg.twist.twist.angular.y = 0;
  msg.twist.twist.angular.z = delta_theta / delta_t;

  pub_odometry_->publish(msg);

  // Actualización de estado
  last_ticks_fl_ = encoder->ticks[0].data;
  last_ticks_fr_ = encoder->ticks[1].data;
  last_ticks_rl_ = encoder->ticks[2].data;
  last_ticks_rr_ = encoder->ticks[3].data;
  last_ticks_time = current_time;

  // Broadcast de TF
  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = current_time;
  t.header.frame_id = "odom";
  t.child_frame_id = "base_link";
  t.transform.translation.x = msg.pose.pose.position.x;
  t.transform.translation.y = msg.pose.pose.position.y;
  t.transform.translation.z = msg.pose.pose.position.z;
  t.transform.rotation = msg.pose.pose.orientation;

  tf_broadcaster_->sendTransform(t);
}

*/


// VIEJO
// VIEJO
// VIEJO
// VIEJO
// VIEJO
// VIEJO

#define WHEEL_BASELINE 0.331
#define WHEEL_RADIUS 0.0975
#define ENCODER_TICKS 500.0


PioneerOdometry::PioneerOdometry() : Node("nodeOdometry"), x_(0), y_(0), theta_(0), ticks_initialized_(false)
{
  // Nos suscribimos a los comandos de velocidad en el tópico "/robot/cmd_vel" de tipo geometry_msgs::Twist
  twist_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", rclcpp::QoS(10), std::bind(&PioneerOdometry::on_velocity_cmd, this, std::placeholders::_1));

  vel_pub_left_ = this->create_publisher<std_msgs::msg::Float64>("/robot/left_wheel/cmd_vel", rclcpp::QoS(10));
  vel_pub_right_ = this->create_publisher<std_msgs::msg::Float64>("/robot/right_wheel/cmd_vel", rclcpp::QoS(10));

  encoder_sub_ =  this->create_subscription<robmovil_msgs::msg::EncoderTicks>("/robot/encoders", rclcpp::QoS(10), std::bind(&PioneerOdometry::on_encoder_ticks, this, std::placeholders::_1));
  pub_odometry_ = this->create_publisher<nav_msgs::msg::Odometry>("/robot/odometry", rclcpp::QoS(10));
  
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
}

void PioneerOdometry::on_velocity_cmd(const geometry_msgs::msg::Twist::SharedPtr twist)
{
  double linear_velocity  = twist->linear.x;

  double linear_velocity  = twist->linear.y;
  double angular_velocity = twist->angular.z;

  double vRight = (linear_velocity + (angular_velocity * WHEEL_BASELINE / 2.0)) / WHEEL_RADIUS;
  double vLeft  = (linear_velocity - (angular_velocity * WHEEL_BASELINE / 2.0)) / WHEEL_RADIUS;

  // publish left velocity
  {
    std_msgs::msg::Float64 msg;
    msg.data = vLeft;

    vel_pub_left_->publish(msg);
  }

  // publish right velocity
  {
    std_msgs::msg::Float64 msg;
    msg.data = vRight;

    vel_pub_right_->publish(msg);
  }
}

void PioneerOdometry::on_encoder_ticks(const robmovil_msgs::msg::EncoderTicks::SharedPtr encoder)
{
  // La primera vez que llega un mensaje de encoders
  // inicializo las variables de estado.
  if (!ticks_initialized_) {
    ticks_initialized_ = true;
    last_ticks_left_ = encoder->ticks_left.data;
    last_ticks_right_ = encoder->ticks_right.data;
    last_ticks_time = encoder->header.stamp;
    return;
  }

  int32_t delta_ticks_left = encoder->ticks_left.data - last_ticks_left_;
  int32_t delta_ticks_right = encoder->ticks_right.data - last_ticks_right_;


  /* Utilizar este delta de tiempo entre momentos */
  rclcpp::Time current_time(encoder->header.stamp);
  double delta_t = (current_time - last_ticks_time).seconds();

  double d_izq = (delta_ticks_left  * WHEEL_RADIUS * 2.0 * M_PI) / ENCODER_TICKS;
  double d_der = (delta_ticks_right * WHEEL_RADIUS * 2.0 * M_PI) / ENCODER_TICKS;
  double d = (d_izq + d_der) / 2.0;

  // calcular el desplazamiento relativo: delta_x, delta_y, delta_theta
  double delta_theta = (d_der - d_izq) / WHEEL_BASELINE;
  double delta_x = d * cos(theta_);
  double delta_y = d * sin(theta_);
  
  //double delta_x = d * cos(theta_ + delta_theta / 2.0);
  //double delta_y = d * sin(theta_ + delta_theta / 2.0);


  /** Utilizar variables globales x_, y_, theta_ definidas en el .h */
  //Calculo la nueva pose
  //x_ += d * cos(theta_);
  //y_ += d * sin(theta_);
  x_ += delta_x;
  y_ += delta_y;
  theta_ += delta_theta;



//VISUALIZACIÓN EN CONSOLA

  double normalized_theta_rad = fmod(theta_, 2.0 * M_PI);
  if (normalized_theta_rad < 0) {
    normalized_theta_rad += 2.0 * M_PI;
  }
  // 2. Convertir a grados
  double theta_en_grados = normalized_theta_rad * 180.0 / M_PI;

  //calculo velocidades
  double linear_velocity = d / delta_t;
  double angular_velocity = delta_theta / delta_t;

  RCLCPP_INFO(this->get_logger(), 
              "Pose -> X: %.2f m, Y: %.2f m, Theta: %.2f deg | Vel -> Lin: %.2f m/s, Ang: %.2f rad/s", 
              x_, y_, theta_en_grados, linear_velocity, angular_velocity);// Construir el mensaje odometry utilizando el esqueleto siguiente:
  nav_msgs::msg::Odometry msg;

//END VISUALIZACIÓN EN CONSOLA


  msg.header.stamp = encoder->header.stamp;
  msg.header.frame_id = "odom";
  msg.child_frame_id = "base_link";

  msg.pose.pose.position.x = x_;
  msg.pose.pose.position.y = y_;
  msg.pose.pose.position.z = 0;

  tf2::Quaternion q;
  q.setRPY(0, 0, theta_);  // roll, pitch, yaw
  msg.pose.pose.orientation = tf2::toMsg(q);

  msg.twist.twist.linear.x = d / delta_t;
  msg.twist.twist.linear.y = 0;
  msg.twist.twist.linear.z = 0;

  msg.twist.twist.angular.x = 0;
  msg.twist.twist.angular.y = 0;
  msg.twist.twist.angular.z = delta_theta / delta_t;

  pub_odometry_->publish( msg );

  // Actualizo las variables de estado

  last_ticks_left_ = encoder->ticks_left.data;
  last_ticks_right_ = encoder->ticks_right.data;
  last_ticks_time = current_time;

  /* Mando tambien un transform usando TF */

  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = this->get_clock()->now();
  t.header.frame_id = "odom";
  t.child_frame_id = "base_link";
  t.transform.translation.x = msg.pose.pose.position.x;
  t.transform.translation.y = msg.pose.pose.position.y;
  t.transform.translation.z = msg.pose.pose.position.z;
  t.transform.rotation = msg.pose.pose.orientation;

  tf_broadcaster_->sendTransform(t);


}
