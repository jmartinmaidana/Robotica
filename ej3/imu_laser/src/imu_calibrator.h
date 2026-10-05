#ifndef __ROBMOVIL_EKF_IMU_CALIBRATOR_H__
#define __ROBMOVIL_EKF_IMU_CALIBRATOR_H__

#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <sensor_msgs/msg/imu.hpp>

namespace robmovil_ekf {
  class IMUCalibrator : public rclcpp::Node
  {
    public:
      IMUCalibrator();

      void on_imu_measurement(const sensor_msgs::msg::Imu& msg);

    private:

      void calculate_bias();

      rclcpp::TimerBase::SharedPtr timer_;
      
      rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
      rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_calib_pub;
      
      /* VARIABLES GLOBALES: Pueden utilizar estas variables y 
       * cualquier otra que necesiten para resolver los ejercicios */
      
      // tiempo de la ultima medicion
      rclcpp::Time time_last_measure_;
      // flag indicando si todavia se esta calibrando
      bool is_calibrating_;
      // bias final calculado, luego del tiempo de calibracion
      tf2::Vector3 bias_;
      // vector auxiliar para acumular velocidades angulares durante el periodo de calibracion
      std::vector<tf2::Vector3> calibration_data_;
      // Orientacion estimada hasta el momento
      tf2::Quaternion orientacion_estimada_;
  };
}

#endif // __ROBMOVIL_EKF_IMU_CALIBRATOR_H__
