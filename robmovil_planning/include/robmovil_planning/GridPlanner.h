#ifndef __PATHPLANNER_H__
#define __PATHPLANNER_H__

#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <robmovil_msgs/msg/trajectory.hpp>
#include <nav_msgs/msg/path.hpp>

namespace robmovil_planning {

class GridPlanner : public rclcpp::Node
{
  public:
    GridPlanner();
    
  protected:
  
    /* Transformacion entre el mapa y el marco inercial de movimiento (odom) */
    geometry_msgs::msg::TransformStamped map_to_odom_;
    
    /* Se almacena el puntero del mensaje /grid recibido
     * NOTA: Si es null entonces se invalidan las funciones auxiliares */
    nav_msgs::msg::OccupancyGrid::SharedPtr grid_;
    
    /* Pose donde inicia el robot (map_to_odom) y pose del goal */
    geometry_msgs::msg::PoseStamped starting_pose_;
    geometry_msgs::msg::PoseStamped goal_pose_;
    
    /* Funcionen auxiliares para operar con la grilla de ocupacion.
     * Las posiciones x,y obtenidas son referentes al marco del mapa (map) */
     
    /* Posicion de la esquina/centro de la celda i,j. 
     * Devuelve false en caso de que los parametros sean invalidos o grid_ = NULL */
    bool getOriginOfCell(uint i, uint j, double& x, double& y);
    bool getCenterOfCell(uint i, uint j, double& x, double& y);
    
    /* Celda correspondiente a la posicion x,y (referente al marco del mapa)
     * Devuelve false en caso de que los parametros sean invalidos o grid_ = NULL */
    bool getCellOfPosition(const double& x, const double& y, uint& i, uint& j);

    /* Funciones de comprobacion de ocupacion, se considera la celda ocupada si
     * posee un valor asignado distinto de 0 (es decir, no se utiliza threshold)
     * Devuelve false en caso de que los parametros sean invalidos o grid_ = NULL */    
    bool isCellOccupy(uint i, uint j);
    bool isANeighborCellOccupy(uint i, uint j);
    bool isPositionOccupy(const double& x, const double& y);
    
    /**
     * Callback que implementaran los planificadores derivados.
     * 
     * @result_trajectory
     *  Trayectoria encontrada entre start y goal.
     * 
     * @return
     *   True si pudo encontrarse un camino entre start y goal. False en caso contrario.
     */
    virtual bool do_planning(robmovil_msgs::msg::Trajectory& result_trajectory);

  private:
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;

    rclcpp::Publisher<robmovil_msgs::msg::Trajectory>::SharedPtr trajectory_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    
    void publishPath(const robmovil_msgs::msg::Trajectory& trajectory);

    tf2_ros::TransformListener transform_listener_;
    tf2_ros::Buffer tf_buffer;

    void on_grid(const nav_msgs::msg::OccupancyGrid::SharedPtr grid);
    void on_goal(const geometry_msgs::msg::PoseStamped::SharedPtr goal);
};

}
#endif // __PATHPLANNER_H__
