// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from robmovil_msgs:msg/EncoderTicks.idl
// generated code does not contain a copyright notice

#ifndef ROBMOVIL_MSGS__MSG__DETAIL__ENCODER_TICKS__BUILDER_HPP_
#define ROBMOVIL_MSGS__MSG__DETAIL__ENCODER_TICKS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "robmovil_msgs/msg/detail/encoder_ticks__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace robmovil_msgs
{

namespace msg
{

namespace builder
{

class Init_EncoderTicks_ticks_right
{
public:
  explicit Init_EncoderTicks_ticks_right(::robmovil_msgs::msg::EncoderTicks & msg)
  : msg_(msg)
  {}
  ::robmovil_msgs::msg::EncoderTicks ticks_right(::robmovil_msgs::msg::EncoderTicks::_ticks_right_type arg)
  {
    msg_.ticks_right = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robmovil_msgs::msg::EncoderTicks msg_;
};

class Init_EncoderTicks_ticks_left
{
public:
  explicit Init_EncoderTicks_ticks_left(::robmovil_msgs::msg::EncoderTicks & msg)
  : msg_(msg)
  {}
  Init_EncoderTicks_ticks_right ticks_left(::robmovil_msgs::msg::EncoderTicks::_ticks_left_type arg)
  {
    msg_.ticks_left = std::move(arg);
    return Init_EncoderTicks_ticks_right(msg_);
  }

private:
  ::robmovil_msgs::msg::EncoderTicks msg_;
};

class Init_EncoderTicks_header
{
public:
  Init_EncoderTicks_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_EncoderTicks_ticks_left header(::robmovil_msgs::msg::EncoderTicks::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_EncoderTicks_ticks_left(msg_);
  }

private:
  ::robmovil_msgs::msg::EncoderTicks msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::robmovil_msgs::msg::EncoderTicks>()
{
  return robmovil_msgs::msg::builder::Init_EncoderTicks_header();
}

}  // namespace robmovil_msgs

#endif  // ROBMOVIL_MSGS__MSG__DETAIL__ENCODER_TICKS__BUILDER_HPP_
