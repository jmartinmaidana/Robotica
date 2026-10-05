#ifndef __ROBMOVIL_EKF_LOCALIZEREKF_H__
#define __ROBMOVIL_EKF_LOCALIZEREKF_H__

#include <rclcpp/rclcpp.hpp>
#include <kalman/ekfilter.hpp>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Matrix3x3.h>

/**
 * TODO: abstraerse de los tipos internos (solo habria que lidiar con eso en los metodos a overridear
 */

namespace robmovil_ekf
{
  typedef Kalman::EKFilter<double,1,false,false,false> EKFilter;

  /**
   * Localizador EKF
   *
   * Estado: X = (x,y,theta)
   * Input: U = (v,w)
   * Medicion: Z = (r,rho)
   * Se incluyen dos ruidos aditivos para modelo de proceso y sensado, respectivamente
   */
  class LocalizerEKF : public EKFilter
  {
    public:
      LocalizerEKF(void);

      /* Jacobiano de f() respecto a X */
      void makeBaseA(void);
      void makeA(void);

      /* Jacobiano de f() respecto a W */
      void makeBaseW(void);
//      void makeW(void);

      /* Covarianza de f() */
      void makeBaseQ(void);

      /* Jacobiano de h() con respecto a X */
      void makeBaseH(void);
      void makeH(void);

      /* Jacobiano de h() con respecto a R */
      void makeBaseV(void);

      /* Covarianza de h() */
      void makeBaseR(void);

      /* modelo de proceso */
      void makeProcess(void);

      /* modelo de sensado */
      void makeMeasure(void);
      
      void set_map(const std::vector<Vector>& observations);
      void set_delta_t(double delta); // seconds
      bool set_measure(const Vector& measured_z);

    private:
    
      /* Encuentra el landmark del mapa mas cercano al provisto y lo devuelve por corresponding_landmark */
      bool find_corresponding_landmark(const tf2::Vector3& measured_landmark, tf2::Vector3& corresponding_landmark, float delta);
      
      /* Convierte una medicion en coordenadas polares (relativa al sistema de coordenadas del robot) a un
       * landmark absoluto en el sistema de coordenadas del mundo */
      tf2::Vector3 measure2landmark(const Vector& measure);
      
      /* Convierte un landmark (en coordenadas cartesianas en referencia al mapa) a una medicion (coordenadas polares)
       *  en referencia al sistema de coordenadas del robot */
      LocalizerEKF::Vector landmark2measure(const tf2::Vector3& landmark);

      /* Mapa compuesto de landmarks en coordenadas cartesianas en referencia 
       * al lugar donde comenzo a funcionar el sistema */
      std::vector<tf2::Vector3> map_landmarks;
      
      /* Delta tiempo desde la ultima prediccion (time update) */
      double delta_t;
      
      /* Landmark del mapa (en coordenadas cartesianas en referencia al lugar donde empezo el sistema)
       * el cual corresponde con la ultima medicion ingresada (por set_measure) */
      tf2::Vector3 correspondence_landmark;
  };
}

#endif // __ROBMOVIL_EKF_LOCALIZEREKF_H__
