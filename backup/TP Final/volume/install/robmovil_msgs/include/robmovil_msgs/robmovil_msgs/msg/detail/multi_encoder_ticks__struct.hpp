// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from robmovil_msgs:msg/MultiEncoderTicks.idl
// generated code does not contain a copyright notice

#ifndef ROBMOVIL_MSGS__MSG__DETAIL__MULTI_ENCODER_TICKS__STRUCT_HPP_
#define ROBMOVIL_MSGS__MSG__DETAIL__MULTI_ENCODER_TICKS__STRUCT_HPP_

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

#ifndef _WIN32
# define DEPRECATED__robmovil_msgs__msg__MultiEncoderTicks __attribute__((deprecated))
#else
# define DEPRECATED__robmovil_msgs__msg__MultiEncoderTicks __declspec(deprecated)
#endif

namespace robmovil_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct MultiEncoderTicks_
{
  using Type = MultiEncoderTicks_<ContainerAllocator>;

  explicit MultiEncoderTicks_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    (void)_init;
  }

  explicit MultiEncoderTicks_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _ticks_type =
    std::vector<int32_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int32_t>>;
  _ticks_type ticks;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__ticks(
    const std::vector<int32_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int32_t>> & _arg)
  {
    this->ticks = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    robmovil_msgs::msg::MultiEncoderTicks_<ContainerAllocator> *;
  using ConstRawPtr =
    const robmovil_msgs::msg::MultiEncoderTicks_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<robmovil_msgs::msg::MultiEncoderTicks_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<robmovil_msgs::msg::MultiEncoderTicks_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      robmovil_msgs::msg::MultiEncoderTicks_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<robmovil_msgs::msg::MultiEncoderTicks_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      robmovil_msgs::msg::MultiEncoderTicks_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<robmovil_msgs::msg::MultiEncoderTicks_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<robmovil_msgs::msg::MultiEncoderTicks_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<robmovil_msgs::msg::MultiEncoderTicks_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__robmovil_msgs__msg__MultiEncoderTicks
    std::shared_ptr<robmovil_msgs::msg::MultiEncoderTicks_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__robmovil_msgs__msg__MultiEncoderTicks
    std::shared_ptr<robmovil_msgs::msg::MultiEncoderTicks_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MultiEncoderTicks_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->ticks != other.ticks) {
      return false;
    }
    return true;
  }
  bool operator!=(const MultiEncoderTicks_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MultiEncoderTicks_

// alias to use template instance with default allocator
using MultiEncoderTicks =
  robmovil_msgs::msg::MultiEncoderTicks_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace robmovil_msgs

#endif  // ROBMOVIL_MSGS__MSG__DETAIL__MULTI_ENCODER_TICKS__STRUCT_HPP_
