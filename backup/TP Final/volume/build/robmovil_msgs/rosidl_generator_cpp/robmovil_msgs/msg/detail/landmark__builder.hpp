// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from robmovil_msgs:msg/Landmark.idl
// generated code does not contain a copyright notice

#ifndef ROBMOVIL_MSGS__MSG__DETAIL__LANDMARK__BUILDER_HPP_
#define ROBMOVIL_MSGS__MSG__DETAIL__LANDMARK__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "robmovil_msgs/msg/detail/landmark__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace robmovil_msgs
{

namespace msg
{

namespace builder
{

class Init_Landmark_bearing
{
public:
  explicit Init_Landmark_bearing(::robmovil_msgs::msg::Landmark & msg)
  : msg_(msg)
  {}
  ::robmovil_msgs::msg::Landmark bearing(::robmovil_msgs::msg::Landmark::_bearing_type arg)
  {
    msg_.bearing = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robmovil_msgs::msg::Landmark msg_;
};

class Init_Landmark_range
{
public:
  Init_Landmark_range()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Landmark_bearing range(::robmovil_msgs::msg::Landmark::_range_type arg)
  {
    msg_.range = std::move(arg);
    return Init_Landmark_bearing(msg_);
  }

private:
  ::robmovil_msgs::msg::Landmark msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::robmovil_msgs::msg::Landmark>()
{
  return robmovil_msgs::msg::builder::Init_Landmark_range();
}

}  // namespace robmovil_msgs

#endif  // ROBMOVIL_MSGS__MSG__DETAIL__LANDMARK__BUILDER_HPP_
