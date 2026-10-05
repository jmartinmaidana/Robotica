// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from robmovil_msgs:msg/EncoderTicks.idl
// generated code does not contain a copyright notice

#ifndef ROBMOVIL_MSGS__MSG__DETAIL__ENCODER_TICKS__TRAITS_HPP_
#define ROBMOVIL_MSGS__MSG__DETAIL__ENCODER_TICKS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "robmovil_msgs/msg/detail/encoder_ticks__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'ticks_left'
// Member 'ticks_right'
#include "std_msgs/msg/detail/int32__traits.hpp"

namespace robmovil_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const EncoderTicks & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: ticks_left
  {
    out << "ticks_left: ";
    to_flow_style_yaml(msg.ticks_left, out);
    out << ", ";
  }

  // member: ticks_right
  {
    out << "ticks_right: ";
    to_flow_style_yaml(msg.ticks_right, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const EncoderTicks & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: ticks_left
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ticks_left:\n";
    to_block_style_yaml(msg.ticks_left, out, indentation + 2);
  }

  // member: ticks_right
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ticks_right:\n";
    to_block_style_yaml(msg.ticks_right, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const EncoderTicks & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace robmovil_msgs

namespace rosidl_generator_traits
{

[[deprecated("use robmovil_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const robmovil_msgs::msg::EncoderTicks & msg,
  std::ostream & out, size_t indentation = 0)
{
  robmovil_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use robmovil_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const robmovil_msgs::msg::EncoderTicks & msg)
{
  return robmovil_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<robmovil_msgs::msg::EncoderTicks>()
{
  return "robmovil_msgs::msg::EncoderTicks";
}

template<>
inline const char * name<robmovil_msgs::msg::EncoderTicks>()
{
  return "robmovil_msgs/msg/EncoderTicks";
}

template<>
struct has_fixed_size<robmovil_msgs::msg::EncoderTicks>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::Header>::value && has_fixed_size<std_msgs::msg::Int32>::value> {};

template<>
struct has_bounded_size<robmovil_msgs::msg::EncoderTicks>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::Header>::value && has_bounded_size<std_msgs::msg::Int32>::value> {};

template<>
struct is_message<robmovil_msgs::msg::EncoderTicks>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ROBMOVIL_MSGS__MSG__DETAIL__ENCODER_TICKS__TRAITS_HPP_
