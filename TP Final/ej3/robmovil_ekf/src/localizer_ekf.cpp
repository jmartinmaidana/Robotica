#include <cmath>
#include <angles/angles.h>
#include "localizer_ekf.h"

#define NEAREST_NEIGHBOR_RADIUS 1.0

robmovil_ekf::LocalizerEKF::LocalizerEKF(void) : EKFilter(3, 2, 3, 2, 2)
{
  delta_t = 0;

  /* estado inicial */
  Vector x0(3);
  x0(1) = 0;
  x0(2) = 0;
  x0(3) = 0;

  /* covarianza inicial */
  Matrix P(3, 3);
  P(1,1) = 0.1;
  P(1,2) = 0;
  P(1,3) = 0;
  P(2,1) = 0;
  P(2,2) = 0.1;
  P(2,3) = 0;
  P(3,1) = 0;
  P(3,2) = 0;
  P(3,3) = 0.1;

  RCLCPP_INFO(rclcpp::get_logger("robmovil_ekf"), "Initial state x0: ( %f, %f, %f )", x0(1), x0(2), x0(3));
  RCLCPP_INFO(rclcpp::get_logger("robmovil_ekf"), "Initial cov P0: [ %f, %f, %f, %f, %f, %f, %f, %f, %f ]", P(1,1), P(1,2), P(1,3), P(2,1), P(2,2), P(2,3), P(3,1), P(3,2), P(3,3));

  /* inicializa el filtro con estado y covarianza iniciales */
  init(x0, P); /* NOTA: esta llamada utiliza las referencias de x0 y P */
}

void robmovil_ekf::LocalizerEKF::set_map(const std::vector<LocalizerEKF::Vector>& observations)
{
  /* Nota: Se asume x = 0, el origen de coordenadas del mapa comienza donde el robot esta ahora,
   * los landmarks se guardar en relacion al mapa */
  for (int i = 0; i < observations.size(); i++)
  {
    tf2::Vector3 landmark = measure2landmark(observations[i]);
    map_landmarks.push_back(landmark);
    
    RCLCPP_INFO(rclcpp::get_logger("robmovil_ekf"), "Landmark: %f, %f, %f", landmark.getX(), landmark.getY(), landmark.getZ());
  }
}

void robmovil_ekf::LocalizerEKF::set_delta_t(double delta)
{
  delta_t = delta;
}

/** Notificacion de una nueva medicion para su actualizacion.
 * 
 *  Se define la variable global correspondence_landmark con las coordenadas cartesianas
 *  del landmark perteneciente al mapa con el que la nueva medicion debera compararse */
bool robmovil_ekf::LocalizerEKF::set_measure(const Vector& new_measure_z)
{
  RCLCPP_INFO(rclcpp::get_logger("robmovil_ekf"), "New measure (polar): %d", new_measure_z);

  /* Convertir medicion actual a landmark, en representation cartesiana y en referencia al mundo
   * (lugar en el momento en que comenzo el sistema) */
  tf2::Vector3 landmark_cartesian = measure2landmark(new_measure_z);
  
  RCLCPP_INFO(rclcpp::get_logger("robmovil_ekf"), "Landmark (cartesian): %f, %f, %f", landmark_cartesian.getX(), landmark_cartesian.getY(), landmark_cartesian.getZ());

  /* Buscar en el mapa el landmark mas cercano a la observacion (establecimiento de correspondencia por nearest-neighbor) */
  bool found_correspondence = find_corresponding_landmark(landmark_cartesian, correspondence_landmark, NEAREST_NEIGHBOR_RADIUS);
  
  if (!found_correspondence)
    RCLCPP_INFO(rclcpp::get_logger("robmovil_ekf"), "No correspondence");
  else
    RCLCPP_INFO(rclcpp::get_logger("robmovil_ekf"), "Correspondence found: %f, %f, %f", correspondence_landmark.getX(), correspondence_landmark.getY(), correspondence_landmark.getZ());
  return found_correspondence;
}

/** Jacobiano de A con respecto del estado (f) */
void robmovil_ekf::LocalizerEKF::makeBaseA(void)
{
  A(1,1) = 1;
  A(1,2) = 0;
  A(2,1) = 0;
  A(2,2) = 1;
  A(3,1) = 0;
  A(3,2) = 0;
  A(3,3) = 1;
}

/** Jacobiano de A con respecto del estado (f)
 *  La matriz se actualiza en cada ciclo de actualizacion de tiempo (prediccion) */
void robmovil_ekf::LocalizerEKF::makeA(void)
{
  /* COMPLETAR: Utilizando variables globales x, u y delta_t */
  
  A(1,3) = 0;
  A(2,3) = 0;

  RCLCPP_INFO(rclcpp::get_logger("robmovil_ekf"), "A: %d", A);
}


/** Jacobiano de W respecto de f */
void robmovil_ekf::LocalizerEKF::makeBaseW(void)
{
  /* COMPLETAR: Con las derivadas del modelo de movimiento (o proceso) con respecto al ruido ADITIVO w */
  
  W(1,1) = 0;
  W(1,2) = 0;
  W(1,3) = 0;
  W(2,1) = 0;
  W(2,2) = 0;
  W(2,3) = 0;
  W(3,1) = 0;
  W(3,2) = 0;
  W(3,3) = 0;


}

/** covarianza de W (ruido en f) */
void robmovil_ekf::LocalizerEKF::makeBaseQ()
{
  Q(1,1) = 0.001; // Var(w1) = 0.1^2 (1 milimetro de desvio estandar)
  Q(1,2) = 0;
  Q(1,3) = 0;
  Q(2,1) = 0;
  Q(2,2) = 0.001; // Var(w2) = 0.1^2
  Q(2,3) = 0;
  Q(3,1) = 0;
  Q(3,2) = 0;
  Q(3,3) = pow(0.02,2); // Var(w3) = 0.1^2
}

/** Jacobiano de H respecto del modelo de sensado (h) (valores iniciales) **/
void robmovil_ekf::LocalizerEKF::makeBaseH(void)
{
  H(1,1) = 0;
  H(1,2) = 0;
  H(1,3) = 0;
  H(2,1) = 0;
  H(2,2) = 0;
  H(2,3) = -1;
}

/** Jacobiano de H respecto del modelo de sensado (h)
 *  La matriz se actualiza en cada ciclo de actualizacion de mediciones */
void robmovil_ekf::LocalizerEKF::makeH(void)
{  
  /* Posicion del robot con respecto al MUNDO, es decir, con respecto 
   * a donde se encontraba cuando comenzo el sistema */
  tf2::Vector3 robot_position(x(1), x(2), 0);
  double robot_orientation = x(3);

  /* Obtener las coordenadas de correspondence_landmark con respecto al robot */
  tf2::Vector3 diff_robot_landmark = correspondence_landmark - robot_position;

  // Coordenadas cartesianas del landmark con respecto al robot
  RCLCPP_INFO(rclcpp::get_logger("robmovil_ekf"), "Relative_landmark: %f, %f, %f", diff_robot_landmark.getX(), diff_robot_landmark.getY(), diff_robot_landmark.getZ());

  if (diff_robot_landmark.length2() < 0.001)
  {
    H(1,1) = H(2,2) = 1;
    H(1,2) = H(2,1) = 0;

    RCLCPP_INFO(rclcpp::get_logger("robmovil_ekf"), "Landmark too close to robot! Fake H used");
  } else {
    
    /* COMPLETAR: Calcular H en base al landmark del mapa relativo al robot */
    
    H(1,1) = 0;
    H(1,2) = 0;
    H(2,1) = 0;
    H(2,2) = 0;

  }

  RCLCPP_INFO(rclcpp::get_logger("robmovil_ekf"), "H: %d", H);
}

/** Jacobiano de H respecto de v **/
void robmovil_ekf::LocalizerEKF::makeBaseV(void)
{
  /* COMPLETAR: Con las derivadas del modelo de sensado con respecto al ruido ADITIVO v */
  
  V(1,1) = 0;
  V(1,2) = 0;
  V(2,1) = 0;
  V(2,2) = 0;
  
  RCLCPP_INFO(rclcpp::get_logger("robmovil_ekf"), "V: %d", V);
}

/** Covarianza de v **/
void robmovil_ekf::LocalizerEKF::makeBaseR()
{
  R(1,1) = 0.001; // Var(rho) = (0.1)^2 (desvio de 10cm)
  R(1,2) = 0;
  R(2,1) = 0;
  R(2,2) = 0.00007615435; // Var(phi) = (0.5 * pi / 180)^2 (desvio de 0.5 grados)
  
  RCLCPP_INFO(rclcpp::get_logger("robmovil_ekf"), "R: %d", R);
}

/** Modelo de movimiento o proceso: x_t = f(x_t-1, u_t-1).
 *  
 *  Se debe utilizar el estado anterior y la entrada del modelo de movimiento
 *  para definir (predecir) la variable x */
void robmovil_ekf::LocalizerEKF::makeProcess(void)
{
  /* COMPLETAR: Utilizar las variables globales x_t-1, u y delta_t 
   * para predecir el estado siguiente (prior state estimate).
   * 
   * Guardar el resultado en la variable global x */
   LocalizerEKF::Vector x_old(x); // X_t-1
   
  x(1) = 0;
  x(2) = 0;
  x(3) = angles::normalize_angle(0);
   

  RCLCPP_INFO(rclcpp::get_logger("robmovil_ekf"), "Process model: X_t-1: %d, X_t: %d, delta_t: %d", x_old, x, delta_t);
}

/** Modelo de sensado: z_t = h(x_t).
 *  
 *  Se debe utilizar la variable global correspondence_landmark previamente definida
 *  para definir la variable z con lo que deberia haber medido el sensor */
void robmovil_ekf::LocalizerEKF::makeMeasure(void)
{
  z = landmark2measure(correspondence_landmark);
  
  RCLCPP_INFO(rclcpp::get_logger("robmovil_ekf"), "Expected measure: %d", z);
}

/** Recibe una medicion de landmark reciente con respecto al origen del mapa (lugar en que comenzo el sistema)
 *  y busca el landmark del mapa más cercano teniendo un radio delta como umbral
 *  
 *  NOTA: El landmark perteneciente al mapa al cual hace referencia el landmark medido debe ser devuelto
 *        por la referencia corresponding_landmark */
bool robmovil_ekf::LocalizerEKF::find_corresponding_landmark(const tf2::Vector3& measured_landmark, tf2::Vector3& corresponding_landmark, float delta_radio)
{
  /* COMPLETAR: Encontrar el landmark del mapa dentro del radio (delta_radio), mas cercano
   * a measured_landmark.
   * 
   * El resultado debe devolverse por la referencia corresponding_landmark */
  
  bool found = false;

  float min_distance = std::numeric_limits<float>::max();
   
  for (int i = 0; i < map_landmarks.size(); i++)
  {
    /* COMPLETAR */
    corresponding_landmark = map_landmarks[i];
    found = false;
    
  }

  return found;
}

/** Convierte una medicion en coordenadas polares (relativa al sistema de coordenadas del robot) a un
 * landmark absoluto en el sistema de coordenadas del mundo */
tf2::Vector3 robmovil_ekf::LocalizerEKF::measure2landmark(const LocalizerEKF::Vector& measure)
{
  /* Toma la medicion realizada en referencia al robot y devuelve
   * las coordenadas cartesianas en referencia al MUNDO.
   * 
   * Al utilizar el ultimo estado ESTIMADO para la conversion, el landmark calculado 
   * corresponde a la prediccion de la posicion del landmark */
  
  /* Measurement parameters */
  double rho = measure(1);
  double phi = measure(2);
  
  /* Ultimo estado estimado del robot */
  double robot_x = x(1);
  double robot_y = x(2);
  double robot_theta = x(3);
  
  /* Considerar la orientacion del robot 
   * y el angulo de la medicion con respecto al robot */

  float absolute_angle = phi + robot_theta;

  tf2::Vector3 predicted_landmark;
  predicted_landmark.setX(rho * cos(absolute_angle));
  predicted_landmark.setY(rho * sin(absolute_angle));
  predicted_landmark.setZ(0);

  /* take robot absolute position into account */
  predicted_landmark = predicted_landmark + tf2::Vector3(robot_x, robot_y, 0);


  return predicted_landmark;
}

/** Convierte un landmark (en coordenadas cartesianas en referencia al mapa) a una medicion (coordenadas polares)
 *  en referencia al sistema de coordenadas del robot */
robmovil_ekf::LocalizerEKF::Vector robmovil_ekf::LocalizerEKF::landmark2measure(const tf2::Vector3& landmark)
{
  /* Toma un landmark del mapa (cartesianas) y devuelve
   * las coordenadas polares con las que deberia haber medido dicho landmark
   * utilizando como posicion actual del robot: la ultima ESTIMACION del estado (x)
   * 
   * NOTA: Para esto se "traduce" la posicion del landmark en respecto
   *       del robot. CONSIDERAR la inversa de la transformacion que va desde el marco
   *       del mundo al marco del robot ( transform_world_robot.inverse() ) */  
  
  RCLCPP_INFO(rclcpp::get_logger("robmovil_ekf"), "Robot pose: %f  %f  %f", x(1), x(2), x(3));
  
  /* Pose del robot contruida utilizando el estado estimado hasta este momento
   * (prior estate estimate) */
  tf2::Transform transform_world_robot;
  transform_world_robot.setOrigin(tf2::Vector3(x(1),x(2),0));
  tf2::Quaternion q;
  q.setRPY(0, 0, x(3));  // roll = 0, pitch = 0, yaw = x(3)
  transform_world_robot.setRotation(q);

  tf2::Vector3 relative_landmark = transform_world_robot.inverse() * landmark;

  LocalizerEKF::Vector measure(2);
  measure(1) = relative_landmark.length(); // Calculo del rho

  relative_landmark.normalize();
  measure(2) = angles::normalize_angle(atan2(relative_landmark.getY(), relative_landmark.getX())); // Calculo del phi


  return measure;
}
