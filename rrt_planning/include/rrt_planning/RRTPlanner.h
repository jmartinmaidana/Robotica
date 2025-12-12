#ifndef __RRTPLANNER_H__
#define __RRTPLANNER_H__

#include <vector>
#include <map>
#include <stdexcept>
#include <algorithm>
#include <initializer_list>
#include <random>
#include <fstream>

#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include "robmovil_planning/GridPlanner.h"

namespace robmovil_planning {

template<size_t CONF_LEN = 3>
class RRTPlanner : public GridPlanner
{
  public:
    RRTPlanner(uint max_iterations, double goal_bias);
    
    double randBetween(double a, double b)
    { std::uniform_real_distribution<double> unif_dist_(a, b); return unif_dist_(rand_gen_); }
    
    struct SpaceConfiguration
    {
      std::vector<double> config;
      
      SpaceConfiguration() : config(CONF_LEN, 0)
      {}
      
      SpaceConfiguration(const SpaceConfiguration& c2) : config(c2.config)
      {}
      
      SpaceConfiguration(std::initializer_list<double> l) : config(l)
      { if(l.size() != CONF_LEN) throw std::invalid_argument( "CONSTRUCT: Too many arguments" ); }
      
      double get(size_t n) const
      { if(n < CONF_LEN) return config[n];
        else throw std::invalid_argument( "GET: Configuration index out of bound" ); }
      
      void set(size_t n, const double& toSet)
      { if(n < CONF_LEN) config[n] = toSet;
        else throw std::invalid_argument( "SET: Configuration index out of bound" ); }
      
      uint length() const
      { return CONF_LEN; }
      
      bool operator <( const SpaceConfiguration& c2) const
      { return std::lexicographical_compare(config.begin(), config.end(), c2.config.begin(), c2.config.end()); }
      
      bool operator ==( const SpaceConfiguration& c2) const
      { return !(*this < c2) and !(c2 < *this); }
      
      bool operator !=( const SpaceConfiguration& c2) const
      { return !(*this == c2); }
      
      std::ostream& print(std::ostream& os) const
      { for(uint i = 0; i < CONF_LEN; i++) if(i < CONF_LEN-1) os << config[i] << " "; else os << config[i];
        return os; }
    };
    
    struct SCWithCost : SpaceConfiguration
    {
      double cost;
      
      SCWithCost(){};
      
      SCWithCost(const SCWithCost& cwc)
      : SpaceConfiguration(cwc), cost(cwc.cost){};
      
      SCWithCost(const SpaceConfiguration& c, const double& _cost)
      : SpaceConfiguration(c), cost(_cost){};
      
      SCWithCost(std::initializer_list<double> l, const double& _cost)
      : SpaceConfiguration(l), cost(_cost){};
    };
    
    class CostCompare
    {
      public:
        bool operator() (const SCWithCost& cwp1, const SCWithCost& cwp2)
          { return cwp1.cost > cwp2.cost; }
    };

  protected:
    double goal_bias_;
    uint max_iterations_;
  
    std::map<SpaceConfiguration, std::list<SpaceConfiguration> > graph_;
    
    SpaceConfiguration start_config_;
    SpaceConfiguration goal_config_;
    
    SpaceConfiguration rand_config_;
    SpaceConfiguration near_config_;
    SpaceConfiguration new_config_;
    
    virtual SpaceConfiguration defineStartConfig() = 0;
    virtual SpaceConfiguration defineGoalConfig() = 0;
    virtual SpaceConfiguration generateRandomConfig() = 0;
    virtual double distancesBetween(const SpaceConfiguration& c1, const SpaceConfiguration& c2) = 0;
    virtual SpaceConfiguration nearest() = 0;
    virtual SpaceConfiguration steer() = 0;
    virtual bool isGoalAchieve() = 0;
    virtual bool isFree() = 0;
    virtual bool isValid() = 0;
    
    virtual bool findPathOnGraph(std::map<SpaceConfiguration, std::list<SpaceConfiguration> >& graph, 
                                 const SpaceConfiguration& src, const SpaceConfiguration& dest,
                                 std::map<SpaceConfiguration, SpaceConfiguration>& came_from) const;
    
    virtual void notifyTrajectory(robmovil_msgs::msg::Trajectory& result_trajectory,
                                  const SpaceConfiguration& start, const SpaceConfiguration& goal, 
                                  std::map<SpaceConfiguration, SpaceConfiguration>& came_from) const = 0;

  private:
    std::mt19937 rand_gen_;
  
    /** Callback override */
    bool do_planning(robmovil_msgs::msg::Trajectory& result_trajectory);
    
    void print_rrt_graph();
};

}

#include "RRTPlanner.tpp"

#endif // __RRTPLANNER_H__
