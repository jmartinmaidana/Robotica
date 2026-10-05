// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from robmovil_msgs:msg/MultiEncoderTicks.idl
// generated code does not contain a copyright notice

#ifndef ROBMOVIL_MSGS__MSG__DETAIL__MULTI_ENCODER_TICKS__TRAITS_HPP_
#define ROBMOVIL_MSGS__MSG__DETAIL__MULTI_ENCODER_TICKS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "robmovil_msgs/msg/detail/multi_encoder_ticks__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace robmovil_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const MultiEncoderTicks & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: ticks
  {
    if (msg.ticks.size() == 0) {
      out << "ticks: []";
    } else {
      out << "ticks: [";
      size_t pending_items = msg.ticks.size();
      for (auto item : msg.ticks) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const MultiEncoderTicks & msg,
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

  // member: ticks
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.ticks.size() == 0) {
      out << "ticks: []\n";
    } else {
      out << "ticks:\n";
      for (auto item : msg.ticks) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const MultiEncoderTicks & msg, bool use_flow_style = false)
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
  const robmovil_msgs::msg::MultiEncoderTicks & msg,
  std::ostream & out, size_t indentation = 0)
{
  robmovil_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use robmovil_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const robmovil_msgs::msg::MultiEncoderTicks & msg)
{
  return robmovil_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<robmovil_msgs::msg::MultiEncoderTicks>()
{
  return "robmovil_msgs::msg::MultiEncoderTicks";
}

template<>
inline const char * name<robmovil_msgs::msg::MultiEncoderTicks>()
{
  return "robmovil_msgs/msg/MultiEncoderTicks";
}

template<>
struct has_fixed_size<robmovil_msgs::msg::MultiEncoderTicks>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<robmovil_msgs::msg::MultiEncoderTicks>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<robmovil_msgs::msg::MultiEncoderTicks>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ROBMOVIL_MSGS__MSG__DETAIL__MULTI_ENCODER_TICKS__TRAITS_HPP_
