#include "TrajectoryFollower.h"

class FeedForwardController : public TrajectoryFollower
{
  public:

    FeedForwardController();

  protected:

    bool control(const rclcpp::Time& t, double& v, double& w) override;
};
