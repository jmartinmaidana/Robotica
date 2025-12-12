#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <robmovil_msgs/msg/trajectory.hpp>

class TrajectoryFollower : public rclcpp::Node
{
  public:

    TrajectoryFollower();

    void stop_timer() { timer_->cancel(); }

  protected:

    /**
     * Callback que implementaran las clases derivadas.
     * 
     * @return
     *   false cuando termina de ejecutar la trayectoria, true en caso contrario.
     */
    virtual bool control(const rclcpp::Time& t, double& v, double& w) = 0;

    const rclcpp::Time& getInitialTime() const
    { return t0_; }

    const robmovil_msgs::msg::Trajectory& getTrajectory() const
    { return current_trajectory_; }

    bool nextPointIndex(const rclcpp::Time& time, size_t&) const;

  private:

    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;

    rclcpp::Subscription<robmovil_msgs::msg::Trajectory>::SharedPtr trajectory_sub_;

    // tiempo en el cuál se comenzó a ejecutar el seguimiento actual.
    rclcpp::Time t0_;

    // trayectoria actual.
    robmovil_msgs::msg::Trajectory current_trajectory_;

  // funciones auxiliares

    void handleNewTrajectory(const robmovil_msgs::msg::Trajectory& trajectory_msg);

    void timerCallback();
};
