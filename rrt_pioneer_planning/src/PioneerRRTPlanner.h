#ifndef __PIONEERRRTPLANNER_H__
#define __PIONEERRRTPLANNER_H__

#include <vector>
#include <map>
#include <stdexcept>
#include <algorithm>
#include <initializer_list>
#include <random>

#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/utils.hpp>

#include "rrt_planning/RRTPlanner.h"

namespace robmovil_planning {

class PioneerRRTPlanner : public robmovil_planning::RRTPlanner<3>
{
  public:
    PioneerRRTPlanner();

  protected:
  
    double time_step_;
    double Vx_step_;
    double Wz_step_;
  
    virtual double distancesBetween(const SpaceConfiguration& c1, const SpaceConfiguration& c2);
  
    SpaceConfiguration defineStartConfig();
    SpaceConfiguration defineGoalConfig();
    
    SpaceConfiguration generateRandomConfig();
    SpaceConfiguration nearest();
    SpaceConfiguration steer();
    
    bool isGoalAchieve();
    bool isFree();
    bool isValid();
    
    void notifyTrajectory(robmovil_msgs::msg::Trajectory& result_trajectory,
                                  const SpaceConfiguration& start, const SpaceConfiguration& goal, 
                                  std::map<SpaceConfiguration, SpaceConfiguration>& came_from) const;

  private:
  
};

}

#endif // __PIONEERRRTPLANNER_H__
