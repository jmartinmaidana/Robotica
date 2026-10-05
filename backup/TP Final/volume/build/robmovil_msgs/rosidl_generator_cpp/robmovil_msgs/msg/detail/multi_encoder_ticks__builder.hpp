// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from robmovil_msgs:msg/MultiEncoderTicks.idl
// generated code does not contain a copyright notice

#ifndef ROBMOVIL_MSGS__MSG__DETAIL__MULTI_ENCODER_TICKS__BUILDER_HPP_
#define ROBMOVIL_MSGS__MSG__DETAIL__MULTI_ENCODER_TICKS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "robmovil_msgs/msg/detail/multi_encoder_ticks__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace robmovil_msgs
{

namespace msg
{

namespace builder
{

class Init_MultiEncoderTicks_ticks
{
public:
  explicit Init_MultiEncoderTicks_ticks(::robmovil_msgs::msg::MultiEncoderTicks & msg)
  : msg_(msg)
  {}
  ::robmovil_msgs::msg::MultiEncoderTicks ticks(::robmovil_msgs::msg::MultiEncoderTicks::_ticks_type arg)
  {
    msg_.ticks = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robmovil_msgs::msg::MultiEncoderTicks msg_;
};

class Init_MultiEncoderTicks_header
{
public:
  Init_MultiEncoderTicks_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MultiEncoderTicks_ticks header(::robmovil_msgs::msg::MultiEncoderTicks::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_MultiEncoderTicks_ticks(msg_);
  }

private:
  ::robmovil_msgs::msg::MultiEncoderTicks msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::robmovil_msgs::msg::MultiEncoderTicks>()
{
  return robmovil_msgs::msg::builder::Init_MultiEncoderTicks_header();
}

}  // namespace robmovil_msgs

#endif  // ROBMOVIL_MSGS__MSG__DETAIL__MULTI_ENCODER_TICKS__BUILDER_HPP_
