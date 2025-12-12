#include <vector>
#include <numeric>      // std::accumulate
#include "imu_calibrator.h"

#define DO_CALIBRATION 1

robmovil_ekf::IMUCalibrator::IMUCalibrator() : Node("imu_calibrator_node")
{
  this->declare_parameter<int>("calibrate", 0);
  int calibrating_time = this->get_parameter("calibrate").as_int();
  
  if(calibrating_time){
    RCLCPP_INFO(this->get_logger(), "Calibrating time!");
    while((this->get_clock()->now().nanoseconds() == 0));
    RCLCPP_INFO(this->get_logger(), "Calibrating time!");
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    RCLCPP_INFO(this->get_logger(), "Calibrating time2!");
    timer_ = this->create_wall_timer(std::chrono::seconds(calibrating_time), std::bind(&IMUCalibrator::calculate_bias, this));
    is_calibrating_ = true; // comenzar calibracion
    RCLCPP_INFO(this->get_logger(), "StartingCalibrating time!");
  }else{
    is_calibrating_ = false;
  }
  bias_ = tf2::Vector3(0,0,0);
  orientacion_estimada_.setValue(0.0, 0.0, 0.0, 1.0);
  
  imu_sub = this->create_subscription<sensor_msgs::msg::Imu>( "/imu", 10, std::bind(&IMUCalibrator::on_imu_measurement, this, std::placeholders::_1));
  imu_calib_pub = this->create_publisher<sensor_msgs::msg::Imu>("/imu_calib", rclcpp::QoS(10));
}

void robmovil_ekf::IMUCalibrator::calculate_bias(){
  is_calibrating_ = false;
  
  // COMPLETAR: Deben calcular el bias de las mediciones acumuladas durante el tiempo de calibracion
  bias_ = tf2::Vector3(0,0,0);

  for (tf2::Vector3 vel_angular : calibration_data_){
    bias_ = bias_ + vel_angular;
  }

  bias_ = bias_ / calibration_data_.size();
  
  
  RCLCPP_INFO(this->get_logger(), "bias: %f %f %f", bias_.getX(), bias_.getY(), bias_.getZ());
}

void robmovil_ekf::IMUCalibrator::on_imu_measurement(const sensor_msgs::msg::Imu& msg)
{
  // Covertimos la velocidad angular del mensaje al tipo tf2::Vector3
  tf2::Vector3 angular_velocity(msg.angular_velocity.x,
                                msg.angular_velocity.y,
                                msg.angular_velocity.z);
  
  // delta de tiempo entre mediciones
  rclcpp::Time current_time(msg.header.stamp);
  double delta_t;
  if (time_last_measure_.nanoseconds() > 0) {
    delta_t = (current_time - time_last_measure_).seconds();
  }
  time_last_measure_ = current_time;

  if ( is_calibrating_ ) {
    RCLCPP_INFO(this->get_logger(), "Calibrando IMU..");
    
    /* COMPLETAR: Se deben acumular las mediciones de imu para calcular el bias (la media).
     * Pueden utilizar la variable globar calibration_data_ definida en el .h */
    calibration_data_.push_back(angular_velocity);
  } else { // si termino el tiempo de calibracion:
    
    // construimos el mensaje nuevo
    sensor_msgs::msg::Imu imu_calib_msg;
    imu_calib_msg = msg;
    
    // COMPLETAR: Corregir la velocidad angular recibida utilizando el bias calculado
    
    tf2::Vector3 vel_sin_bias = angular_velocity - bias_;

    RCLCPP_INFO(this->get_logger(), "Bias: vel_x: %f , vel_y: %f , vel_z: %f", angular_velocity.getX(), angular_velocity.getY(), angular_velocity.getZ());
    RCLCPP_INFO(this->get_logger(), "Sin Bias: vel_x: %f , vel_y: %f , vel_z: %f", vel_sin_bias.getX(), vel_sin_bias.getY(), vel_sin_bias.getZ());
    
    //double delta_theta = delta_t * vel_sin_bias;

    // COMPLETAR: Integrar la velocidad angular corregida durante un intervalo de tiempo

    tf2::Quaternion delta_q;
    delta_q.setRPY(0, 0, vel_sin_bias.getZ() * delta_t);

    
    orientacion_estimada_ *= delta_q;
    orientacion_estimada_.normalize();
    
    /* Publicacion de las mediciones calibradas */
    imu_calib_msg.angular_velocity.x = vel_sin_bias.getX();
    imu_calib_msg.angular_velocity.y = vel_sin_bias.getY();
    imu_calib_msg.angular_velocity.z = vel_sin_bias.getZ();

    imu_calib_msg.orientation.x = orientacion_estimada_.x();
    imu_calib_msg.orientation.y = orientacion_estimada_.y();
    imu_calib_msg.orientation.z = orientacion_estimada_.z();
    imu_calib_msg.orientation.w = orientacion_estimada_.w();
    imu_calib_pub->publish(imu_calib_msg);
    
  }
}
