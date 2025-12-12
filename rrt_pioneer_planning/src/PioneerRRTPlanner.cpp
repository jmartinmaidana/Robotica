#include "PioneerRRTPlanner.h"

#include <angles/angles.h>
#include <queue>
#include <map>
#include <vector>
#include <random>
#include <queue>

typedef robmovil_planning::PioneerRRTPlanner::SpaceConfiguration SpaceConfiguration;

robmovil_planning::PioneerRRTPlanner::PioneerRRTPlanner()
: RRTPlanner(0, 0)
{ 
  goal_bias_ = this->declare_parameter<double>("goal_bias", 0.6);
  int it_tmp = this->declare_parameter<int>("max_iterations", 20000);
  max_iterations_ = it_tmp >= 0 ? it_tmp : 20000;
  Vx_step_ = this->declare_parameter<double>("linear_velocity_stepping", 0.05);
  Wz_step_ = this->declare_parameter<double>("angular_velocity_stepping", 0.025);
}

SpaceConfiguration robmovil_planning::PioneerRRTPlanner::defineStartConfig()
{
  /* Se toma la variable global de la pose incial y se la traduce a SpaceConfiguration */
  //return SpaceConfiguration( { starting_pose_.getOrigin().getX(), starting_pose_.getOrigin().getY(), tf::getYaw(starting_pose_.getRotation()) } );
  return SpaceConfiguration( { starting_pose_.pose.position.x, starting_pose_.pose.position.y, tf2::getYaw(starting_pose_.pose.orientation) } );
}

SpaceConfiguration robmovil_planning::PioneerRRTPlanner::defineGoalConfig()
{
  /* Se toma la variable global de la pose del goal y se la traduce a SpaceConfiguration */
  //return SpaceConfiguration( { goal_pose_.getOrigin().getX(), goal_pose_.getOrigin().getY(), tf::getYaw(goal_pose_.getRotation()) } );
  return SpaceConfiguration( { goal_pose_.pose.position.x, goal_pose_.pose.position.y, tf2::getYaw(goal_pose_.pose.orientation) } );
}

SpaceConfiguration robmovil_planning::PioneerRRTPlanner::generateRandomConfig()
{  
  
    /* COMPLETAR: Deben retornar una configuracion aleatoria dentro del espacio de busqueda.
     * 
     * ATENCION: - Tener encuenta el valor de la variable global goal_bias_ 
     *           - Pueden utilizar la funcion randBetween(a,b) para la generacion de numeros aleatorios 
     *           - Utilizar las funciones getOriginOfCell() y la informacion de la grilla para establecer el espacio de busqueda:
     *                grid_->info.width, grid_->info.height, grid_->info.resolution */  
     
    double r = randBetween(0,1);
    
    SpaceConfiguration rand( {r, r, r} );


    return rand;
}

double robmovil_planning::PioneerRRTPlanner::distancesBetween(const SpaceConfiguration& c1, const SpaceConfiguration& c2)
{
  /* COMPLETAR: Funcion auxiliar recomendada para evaluar la distancia entre configuraciones
   * 
   * ATENCION: Utilizar abs( angles::shortest_angular_distance(c1.get(2), c2.get(2)) )
   *           para medir la distancia entre las orientaciones */
  
  double dist_ori = abs( angles::shortest_angular_distance(c1.get(2), c2.get(2)) );
  
  return 0;
}

SpaceConfiguration robmovil_planning::PioneerRRTPlanner::nearest()
{
  /* COMPLETAR: Retornar configuracion mas cercana a la aleatoria (rand_config_). DEBE TENER HIJOS LIBRES
   * 
   * ATENCION: - Deberan recorrer la variable global graph_ la cual contiene los nodos del arbol
   *             generado hasta el momento como CLAVES DEL DICCIONARIO
   *           - Se recomienda establecer una relacion de distancia entre configuraciones en distancesBetween() 
   *             y utilizar esa funcion como auxiliar */
  
  SpaceConfiguration nearest;
  double min_distance = std::numeric_limits<double>::max(); 
  
  for(const auto& config : graph_ )
  {
    double d = distancesBetween(config.first, rand_config_);
    
    
  }

  return nearest;
}

SpaceConfiguration robmovil_planning::PioneerRRTPlanner::steer()
{  
  /* COMPLETAR: Retornar una nueva configuracion a partir de la mas cercana near_config_.
   *            La nueva configuracion debe ser ademas la mas cercana a rand_config_ de entre las posibles.
   * 
   * ATENCION: - Utilizar las variables globales Vx_step_ , Wz_step_ para la aplicaciones de las velocidades
   *           - Pensar en la conversion de coordenadas polares a cartesianas al establecer la nueva configuracion
   *           - Utilizar angles::normalize_angle() */
  
  /* Ejemplo de como construir una posible configuracion: */
   double x_posible = near_config_.get(0) /* + algo */;
   double y_posible = near_config_.get(0) /* + algo */;
   double theta_posible = angles::normalize_angle(near_config_.get(2) /* + algo */ );
  
   SpaceConfiguration s_posible({ x_posible, y_posible, theta_posible });
  
   SpaceConfiguration steer;
  
  // /* Conjunto de steers ya ocupados en la configuracion near_config_ */
   const std::list<SpaceConfiguration> occupied_steerings = graph_[near_config_];
   std::vector<SpaceConfiguration> free_steerings;
  
  /* RECOMENDACION: Establecer configuraciones posibles en free_steerings y calcular la mas cercana a rand_config_ */
  
  return steer;
}

bool robmovil_planning::PioneerRRTPlanner::isFree()
{
  /* COMPLETAR: Utilizar la variable global new_config_ para establecer si existe un area segura alrededor de esta */
  
  return false;
}

bool robmovil_planning::PioneerRRTPlanner::isGoalAchieve()
{
  
  /* COMPLETAR: Comprobar si new_config_ se encuentra lo suficientemente cerca del goal.
   * 
   * ATENCION: Utilizar abs( angles::shortest_angular_distance(c1.get(2), c2.get(2)) )
   *           para medir la distancia entre las orientaciones */

  
  return false;
}



/* DESDE AQUI YA NO HACE FALTA COMPLETAR */



bool robmovil_planning::PioneerRRTPlanner::isValid()
{ return true; }

void robmovil_planning::PioneerRRTPlanner::notifyTrajectory(robmovil_msgs::msg::Trajectory& result_trajectory,
                                                            const SpaceConfiguration& start, const SpaceConfiguration& goal, 
                                                            std::map<SpaceConfiguration, SpaceConfiguration>& came_from) const
{
  std::vector<SpaceConfiguration> path;
  SpaceConfiguration current = goal;
  
  path.push_back(current);
  
  while(current != start)
  {
    current = came_from[current];
    path.push_back(current);
  }
  
  result_trajectory.header.stamp = this->now();
  result_trajectory.header.frame_id = "map";
  
  rclcpp::Duration t_from_start = rclcpp::Duration::from_seconds(0.0);
  rclcpp::Duration delta_t = rclcpp::Duration::from_seconds(1.0);

  /* Se recorre de atras para adelante */
  for (auto it = path.rbegin(); it != path.rend(); ++it) {    
    double config_x = it->get(0);
    double config_y = it->get(1);
    double config_theta = it->get(2);
    
    tf2::Transform wp_odom_ref;
    wp_odom_ref.setOrigin(tf2::Vector3(config_x, config_y, 0.0));
    tf2::Quaternion delta_q;
    delta_q.setRPY(0,0,config_theta);
    wp_odom_ref.setRotation(delta_q);
    
    //wp_odom_ref = map_to_odom_.inverse() * wp_odom_ref;
    tf2::Transform tf_map_to_odom;
    tf2::fromMsg(map_to_odom_.transform, tf_map_to_odom);

    wp_odom_ref = tf_map_to_odom.inverse() * wp_odom_ref;
    
    // Se crean los waypoints de la trayectoria
    robmovil_msgs::msg::TrajectoryPoint point_msg;
    
    //transformTFToMsg(wp_odom_ref, point_msg.transform);
    point_msg.transform = tf2::toMsg(wp_odom_ref);
    
    if(it != path.rend()-1) {
      double config_dx = (it+1)->get(0) - config_x;
      double config_dy = (it+1)->get(1) - config_y;
      double config_dtheta = angles::shortest_angular_distance(config_theta, (it+1)->get(2));
      point_msg.velocity.linear.x = sqrt(pow(config_dx,2) + pow(config_dy,2));
      point_msg.velocity.angular.z = config_dtheta;
    }else{
      point_msg.velocity.linear.x = 0;
      point_msg.velocity.angular.z = 0;
    }
    
    point_msg.time_from_start = t_from_start;
    
    result_trajectory.points.push_back( point_msg );
    
    t_from_start = t_from_start + delta_t;
  }
}
