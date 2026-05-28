// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from robmovil_msgs:msg/TrajectoryPoint.idl
// generated code does not contain a copyright notice

#ifndef ROBMOVIL_MSGS__MSG__DETAIL__TRAJECTORY_POINT__BUILDER_HPP_
#define ROBMOVIL_MSGS__MSG__DETAIL__TRAJECTORY_POINT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "robmovil_msgs/msg/detail/trajectory_point__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace robmovil_msgs
{

namespace msg
{

namespace builder
{

class Init_TrajectoryPoint_time_from_start
{
public:
  explicit Init_TrajectoryPoint_time_from_start(::robmovil_msgs::msg::TrajectoryPoint & msg)
  : msg_(msg)
  {}
  ::robmovil_msgs::msg::TrajectoryPoint time_from_start(::robmovil_msgs::msg::TrajectoryPoint::_time_from_start_type arg)
  {
    msg_.time_from_start = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robmovil_msgs::msg::TrajectoryPoint msg_;
};

class Init_TrajectoryPoint_acceleration
{
public:
  explicit Init_TrajectoryPoint_acceleration(::robmovil_msgs::msg::TrajectoryPoint & msg)
  : msg_(msg)
  {}
  Init_TrajectoryPoint_time_from_start acceleration(::robmovil_msgs::msg::TrajectoryPoint::_acceleration_type arg)
  {
    msg_.acceleration = std::move(arg);
    return Init_TrajectoryPoint_time_from_start(msg_);
  }

private:
  ::robmovil_msgs::msg::TrajectoryPoint msg_;
};

class Init_TrajectoryPoint_velocity
{
public:
  explicit Init_TrajectoryPoint_velocity(::robmovil_msgs::msg::TrajectoryPoint & msg)
  : msg_(msg)
  {}
  Init_TrajectoryPoint_acceleration velocity(::robmovil_msgs::msg::TrajectoryPoint::_velocity_type arg)
  {
    msg_.velocity = std::move(arg);
    return Init_TrajectoryPoint_acceleration(msg_);
  }

private:
  ::robmovil_msgs::msg::TrajectoryPoint msg_;
};

class Init_TrajectoryPoint_transform
{
public:
  Init_TrajectoryPoint_transform()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_TrajectoryPoint_velocity transform(::robmovil_msgs::msg::TrajectoryPoint::_transform_type arg)
  {
    msg_.transform = std::move(arg);
    return Init_TrajectoryPoint_velocity(msg_);
  }

private:
  ::robmovil_msgs::msg::TrajectoryPoint msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::robmovil_msgs::msg::TrajectoryPoint>()
{
  return robmovil_msgs::msg::builder::Init_TrajectoryPoint_transform();
}

}  // namespace robmovil_msgs

#endif  // ROBMOVIL_MSGS__MSG__DETAIL__TRAJECTORY_POINT__BUILDER_HPP_
