#include <angles/angles.h>
#include "KinematicPositionController.h"



KinematicPositionController::KinematicPositionController() :
  TrajectoryFollower(), tfBuffer_(this->get_clock()),transform_listener_( tfBuffer_ )
{
    rclcpp::QoS qos_profile(rclcpp::KeepLast(50));
    qos_profile.reliable();
    qos_profile.durability_volatile();

    expected_position_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", rclcpp::QoS(10));

    // Comentamos la suscripcion a la odometria porque usaremos TF segun consigna 3.b
    // current_pos_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/robot/odometry", rclcpp::QoS(10), std::bind(&KinematicPositionController::getCurrentPoseFromOdometry, this, std::placeholders::_1));
          
    std::string goal_selection = this->declare_parameter("goal_selection", "PURSUIT_BASED");
    fixed_goal_x_ = this->declare_parameter("fixed_goal_x", 2.0);
    fixed_goal_y_ = this->declare_parameter("fixed_goal_y", 1.0);
    fixed_goal_a_ = this->declare_parameter("fixed_goal_a", 1.5);
    
    if(goal_selection == "TIME_BASED")
      goal_selection_ = TIME_BASED;
    else if(goal_selection == "PURSUIT_BASED")
      goal_selection_ = PURSUIT_BASED;
    else if(goal_selection == "FIXED_GOAL")
      goal_selection_ = FIXED_GOAL;
    else
      goal_selection_ = TIME_BASED; // default
}

double lineal_interp(const rclcpp::Time& t0, const rclcpp::Time& t1, double y0, double y1, const rclcpp::Time& t)
{
  return y0 + (t - t0).seconds() * (y1 - y0) / (t1 - t0).seconds();
}

void KinematicPositionController::getCurrentPoseFromOdometry(const nav_msgs::msg::Odometry& odometry_msg)
{
  x = odometry_msg.pose.pose.position.x;
  y = odometry_msg.pose.pose.position.y;
  tf2::Quaternion q(odometry_msg.pose.pose.orientation.x,
                    odometry_msg.pose.pose.orientation.y,
                    odometry_msg.pose.pose.orientation.z,
                    odometry_msg.pose.pose.orientation.w);
  double roll, pitch, yaw;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  a = yaw;
}

/**
 * NOTA: Para un sistema estable mantener:
 * - 0 < K_RHO
 * - K_RHO < K_ALPHA
 * - K_BETA < 0
 */
// #define K_RHO 0.65 //por default 1, valor optimo 0.65
// #define K_ALPHA 1.5
// #define K_BETA -0.5
// #define K_THETA 0.5
// #define TOL 0.001

#define K_PX 1.
#define K_PY 1.
#define K_PTHETA 1.

#define LOOKAHEAD 0.5
int last_idx = 0;

bool KinematicPositionController::control(const rclcpp::Time& t, double& vx, double& vy, double& wz)
{
  // Se obtiene la pose actual publicada por la transformacion map -> base_link
  double current_x, current_y, current_a;
  try {
    geometry_msgs::msg::TransformStamped transform = tfBuffer_.lookupTransform("map", "base_link", tf2::TimePointZero);
    current_x = transform.transform.translation.x;
    current_y = transform.transform.translation.y;
    current_a = tf2::getYaw(transform.transform.rotation);
    
    // Actualizamos variables internas para los demas metodos (como getPursuitBasedGoal)
    this->x = current_x; this->y = current_y; this->a = current_a;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(this->get_logger(), "Esperando transformacion map -> base_link: %s", ex.what());
    return true; // Retornamos true para no cancelar la trayectoria, solo esperamos
  }

  // Se obtiene la pose objetivo actual a seguir
  double goal_x, goal_y, goal_a;
  if( not getCurrentGoal(t, goal_x, goal_y, goal_a) )
    return false;
  publishCurrentGoal(t, goal_x, goal_y, goal_a); // publicación de la pose objetivo para visualizar en RViz

  /** EJERCICIO 1: COMPLETAR: Aqui deberan realizar las cuentas necesarias para determinar:
   *             - la velocidad lineal: asignando la variable v
   *             - la velocidad angular: asignando la variable w 
   *  
   *  RECORDAR: cambiar el marco de referencia en que se encuentran dx, dy y theta */

  
  double dx = goal_x - current_x;
  double dy = goal_y - current_y;
  double dtheta = angles::normalize_angle(goal_a - current_a); // VER SI ESTA RESTA ES ASI O AL REVES
  // double ex = dx * cos(goal_a) + dy * sin(goal_a);
  // double ey = -dx * sin(goal_a) + dy * cos(goal_a);
  double ex = dx * cos(current_a) + dy * sin(current_a);
  double ey = -dx * sin(current_a) + dy * cos(current_a);
  

  // Computar variables del sistema de control

  //DIFERENCIAL
  // double rho = sqrt(dx*dx + dy*dy);
  // double alpha = angles::normalize_angle(atan2(dy,dx)-theta); // Normalizes the angle to be -M_PI circle to +M_PI circle It takes and returns radians. 
  // double beta =  angles::normalize_angle((-theta)-alpha); // Realizar el calculo dentro del metodo de normalizacion
  
  vx = K_PX * ex;
  vy = K_PY * ey;
  wz  = K_PTHETA * dtheta;



  // if (TOL > rho){
  //   v = 0;
  //   w = (K_ALPHA*alpha+K_BETA*beta) * K_THETA; 
  // } else {
  //   /* Calcular velocidad lineal y angular* 
  //   * Existen constantes definidas al comienzo del archivo para
  //   * K_RHO, K_ALPHA, K_BETA */
  //   v = K_RHO*rho;
  //   w = K_ALPHA*alpha+K_BETA*beta;
  // }
  

  
    RCLCPP_INFO(this->get_logger(), "ex: %.2f, ey: %.2f, dtheta: %.2f, vx: %.2f, vy: %.2f, wz: %.2f",
            ex, ey, dtheta, vx, vy, wz);

  RCLCPP_INFO(this->get_logger(), "goal_x: %.2f, goal_y: %.2f, goal_a: %.2f, current_x: %.2f, current_y: %.2f, current_a: %.2f",
            goal_x, goal_y, goal_a, current_x, current_y, current_a);

  return true;
}

/* Funcion auxiliar para calcular la distancia euclidea */
double dist2(double x0, double y0, double x1, double y1)
{ return sqrt((x1-x0)*(x1-x0) + (y1-y0)*(y1-y0));}


bool KinematicPositionController::getPursuitBasedGoal(const rclcpp::Time& t, double& x, double& y, double& a)
{
  // Pose actual del robot
  double current_x = this->x;
  double current_y = this->y;
  double current_a = this->a;

  // Trajectory cargada
  const robmovil_msgs::msg::Trajectory& trajectory = getTrajectory();

  if (trajectory.points.empty())
    return false;

  double min_dist = std::numeric_limits<double>::max();
  int index_closest = 0;

  for (unsigned int i = 0; i < trajectory.points.size(); i++) {
    const robmovil_msgs::msg::TrajectoryPoint& wpoint = trajectory.points[i];
    double wpoint_x = wpoint.transform.translation.x;
    double wpoint_y = wpoint.transform.translation.y;

    double d = dist2(current_x, current_y, wpoint_x, wpoint_y);
    if (d < min_dist) {
      min_dist = d;
      index_closest = i;
    }
  }

  for (unsigned int i = 0; i < trajectory.points.size(); i++) {
  
    unsigned int circular_idx = (index_closest + i) % trajectory.points.size();

    const robmovil_msgs::msg::TrajectoryPoint& wpoint = trajectory.points[circular_idx];
    double wpoint_x = wpoint.transform.translation.x;
    double wpoint_y = wpoint.transform.translation.y;
    double wpoint_a = tf2::getYaw(wpoint.transform.rotation);

    double d = dist2(current_x, current_y, wpoint_x, wpoint_y);
    if (d >= LOOKAHEAD) {
      x = wpoint_x;
      y = wpoint_y;
      a = wpoint_a;
      return true;
    }
  }

  // --- 3. Si no hay más puntos adelante, usar el último punto ---
  const robmovil_msgs::msg::TrajectoryPoint& last_wpoint = trajectory.points.back(); 
  x = last_wpoint.transform.translation.x;
  y = last_wpoint.transform.translation.y;
  a = tf2::getYaw(last_wpoint.transform.rotation);

  return true;
}


bool KinematicPositionController::getTimeBasedGoal(const rclcpp::Time& t, double& x, double& y, double& a)
{
  size_t next_point_idx;

  if( not nextPointIndex(t, next_point_idx ) )
    return false;
    
  RCLCPP_INFO(this->get_logger(), "processing index: %zu", next_point_idx);

  const robmovil_msgs::msg::TrajectoryPoint& prev_point = getTrajectory().points[ next_point_idx-1 ];
  const robmovil_msgs::msg::TrajectoryPoint& next_point = getTrajectory().points[ next_point_idx ];

  const rclcpp::Time& t0 = getInitialTime() + prev_point.time_from_start;
  const rclcpp::Time& t1 = getInitialTime() + next_point.time_from_start;

  assert(t0 <= t);
  assert(t < t1);
  double x0 = prev_point.transform.translation.x;
  double x1 = next_point.transform.translation.x;

  double y0 = prev_point.transform.translation.y;
  double y1 = next_point.transform.translation.y;

  double a0 = tf2::getYaw(prev_point.transform.rotation);
  double a1 = tf2::getYaw(next_point.transform.rotation);

  x = lineal_interp(t0, t1, x0, x1, t);
  y = lineal_interp(t0, t1, y0, y1, t);
  a = lineal_interp(t0, t1, a0, a1, t);

  return true;
}
