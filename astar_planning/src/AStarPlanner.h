#ifndef __ASTARPLANNER_H__
#define __ASTARPLANNER_H__

#include <rclcpp/rclcpp.hpp>
#include <robmovil_planning/GridPlanner.h>
#include <vector>

namespace robmovil_planning {

class AStarPlanner : public GridPlanner
{
  public:
    AStarPlanner();
    
    struct Cell
    {
      uint i;
      uint j;
      
      Cell(){};
      Cell(const Cell& c) : i(c.i), j(c.j){};
      Cell(uint _i, uint _j) : i(_i), j(_j){};
      
      bool operator <( const Cell& c2) const
      { return (j < c2.j) or (i < c2.i and j == c2.j); } // Primero por filas (y), luego por columnas (x)
      
      bool operator ==( const Cell& c2) const
      { return j == c2.j and i == c2.i; }
      
      bool operator !=( const Cell& c2) const
      { return j != c2.j or i != c2.i; }
    };

    struct CellWithPriority : Cell
    {
      double priority;
      
      CellWithPriority(){};
      
      CellWithPriority(const Cell& c, const double& _priority)
      : Cell(c), priority(_priority){};
      
      CellWithPriority(uint _i, uint _j, const double& _priority)
      : Cell(_i, _j), priority(_priority){};
    };
    
    class PriorityCompare
    {
      public:
        bool operator() (const CellWithPriority& cwp1, const CellWithPriority& cwp2)
          { return cwp1.priority > cwp2.priority; }
    };
    
  protected:    
    /** Callback override */
    bool do_planning(robmovil_msgs::msg::Trajectory& result_trajectory);
    
    std::vector<Cell> neighbors(const Cell& c);
    
    double heuristic_cost(const Cell& start, const Cell& goal, const Cell& current);
    
    void notifyTrajectory(robmovil_msgs::msg::Trajectory& result_trajectory, const Cell& start, const Cell& goal, 
                          std::map<Cell, Cell>& came_from);
};

}
#endif // __ASTARPLANNER_H__
