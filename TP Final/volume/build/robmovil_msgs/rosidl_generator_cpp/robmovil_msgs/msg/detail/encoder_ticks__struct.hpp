// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from robmovil_msgs:msg/EncoderTicks.idl
// generated code does not contain a copyright notice

#ifndef ROBMOVIL_MSGS__MSG__DETAIL__ENCODER_TICKS__STRUCT_HPP_
#define ROBMOVIL_MSGS__MSG__DETAIL__ENCODER_TICKS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"
// Member 'ticks_left'
// Member 'ticks_right'
#include "std_msgs/msg/detail/int32__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__robmovil_msgs__msg__EncoderTicks __attribute__((deprecated))
#else
# define DEPRECATED__robmovil_msgs__msg__EncoderTicks __declspec(deprecated)
#endif

namespace robmovil_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct EncoderTicks_
{
  using Type = EncoderTicks_<ContainerAllocator>;

  explicit EncoderTicks_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    ticks_left(_init),
    ticks_right(_init)
  {
    (void)_init;
  }

  explicit EncoderTicks_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    ticks_left(_alloc, _init),
    ticks_right(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _ticks_left_type =
    std_msgs::msg::Int32_<ContainerAllocator>;
  _ticks_left_type ticks_left;
  using _ticks_right_type =
    std_msgs::msg::Int32_<ContainerAllocator>;
  _ticks_right_type ticks_right;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__ticks_left(
    const std_msgs::msg::Int32_<ContainerAllocator> & _arg)
  {
    this->ticks_left = _arg;
    return *this;
  }
  Type & set__ticks_right(
    const std_msgs::msg::Int32_<ContainerAllocator> & _arg)
  {
    this->ticks_right = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    robmovil_msgs::msg::EncoderTicks_<ContainerAllocator> *;
  using ConstRawPtr =
    const robmovil_msgs::msg::EncoderTicks_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<robmovil_msgs::msg::EncoderTicks_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<robmovil_msgs::msg::EncoderTicks_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      robmovil_msgs::msg::EncoderTicks_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<robmovil_msgs::msg::EncoderTicks_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      robmovil_msgs::msg::EncoderTicks_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<robmovil_msgs::msg::EncoderTicks_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<robmovil_msgs::msg::EncoderTicks_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<robmovil_msgs::msg::EncoderTicks_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__robmovil_msgs__msg__EncoderTicks
    std::shared_ptr<robmovil_msgs::msg::EncoderTicks_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__robmovil_msgs__msg__EncoderTicks
    std::shared_ptr<robmovil_msgs::msg::EncoderTicks_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const EncoderTicks_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->ticks_left != other.ticks_left) {
      return false;
    }
    if (this->ticks_right != other.ticks_right) {
      return false;
    }
    return true;
  }
  bool operator!=(const EncoderTicks_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct EncoderTicks_

// alias to use template instance with default allocator
using EncoderTicks =
  robmovil_msgs::msg::EncoderTicks_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace robmovil_msgs

#endif  // ROBMOVIL_MSGS__MSG__DETAIL__ENCODER_TICKS__STRUCT_HPP_
