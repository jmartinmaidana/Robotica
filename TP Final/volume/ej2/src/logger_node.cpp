#include <fstream>
#include <rclcpp/rclcpp.hpp>
#include <tf2/utils.h>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

std::string formatTime(const boost::posix_time::ptime& time, const char* format)
{
  boost::posix_time::time_facet* facet = new boost::posix_time::time_facet();
  facet->format( format );

  std::stringstream stream;
  stream.str("");
  stream.imbue(std::locale(std::locale::classic(), facet));
  stream << time;

  return stream.str();
}

std::string timestamp()
{
  return formatTime(boost::posix_time::second_clock::local_time(), "%Y-%m-%d_%H:%M:%S");
}

class Logger : public rclcpp::Node
{
  public:

    Logger();

  private:

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr robot_pose_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr ground_truth_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_poses_sub_;

    std::ofstream robot_logfile_, ground_truth_logfile_, goal_poses_logfile_;

  // funciones auxiliares

    void handleRobotPose(const nav_msgs::msg::Odometry& msg);

    void handleGroundTruthPose(const nav_msgs::msg::Odometry& msg);

    void handleGoalPose(const geometry_msgs::msg::PoseStamped& msg);
};

Logger::Logger() : Node ("nodeLogger"),
  robot_logfile_( timestamp() + "_poses.log" ), ground_truth_logfile_( timestamp() + "_ground-truth.log" ), goal_poses_logfile_( timestamp() + "_goals.log" )
{
  robot_pose_sub_ =  this->create_subscription<nav_msgs::msg::Odometry>("/robot/odometry", rclcpp::QoS(10), std::bind(&Logger::handleRobotPose, this, std::placeholders::_1));
  ground_truth_sub_ =  this->create_subscription<nav_msgs::msg::Odometry>("/robot/ground_truth", rclcpp::QoS(10), std::bind(&Logger::handleGroundTruthPose, this, std::placeholders::_1));
  goal_poses_sub_ =  this->create_subscription<geometry_msgs::msg::PoseStamped>("/goal_pose", rclcpp::QoS(10), std::bind(&Logger::handleGoalPose, this, std::placeholders::_1));
}

void Logger::handleRobotPose(const nav_msgs::msg::Odometry& msg)
{
  robot_logfile_ << msg.header.stamp.sec << " " << msg.pose.pose.position.x << " " << msg.pose.pose.position.y << " " << tf2::getYaw( msg.pose.pose.orientation ) << std::endl;
}

void Logger::handleGroundTruthPose(const nav_msgs::msg::Odometry& msg)
{
  ground_truth_logfile_ << msg.header.stamp.sec << " " << msg.pose.pose.position.x << " " << msg.pose.pose.position.y << " " << tf2::getYaw( msg.pose.pose.orientation ) << std::endl;
}

void Logger::handleGoalPose(const geometry_msgs::msg::PoseStamped& msg)
{
  goal_poses_logfile_ << msg.header.stamp.sec << " " << msg.pose.position.x << " " << msg.pose.position.y << " " << tf2::getYaw( msg.pose.orientation ) << std::endl;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);//, "logger");

  auto logger = std::make_shared<Logger>();
  rclcpp::spin(logger);
  rclcpp::shutdown();
  return 0;
  
}
