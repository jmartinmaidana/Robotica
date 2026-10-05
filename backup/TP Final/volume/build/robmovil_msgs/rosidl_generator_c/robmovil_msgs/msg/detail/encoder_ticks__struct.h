// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from robmovil_msgs:msg/EncoderTicks.idl
// generated code does not contain a copyright notice

#ifndef ROBMOVIL_MSGS__MSG__DETAIL__ENCODER_TICKS__STRUCT_H_
#define ROBMOVIL_MSGS__MSG__DETAIL__ENCODER_TICKS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'ticks_left'
// Member 'ticks_right'
#include "std_msgs/msg/detail/int32__struct.h"

/// Struct defined in msg/EncoderTicks in the package robmovil_msgs.
typedef struct robmovil_msgs__msg__EncoderTicks
{
  std_msgs__msg__Header header;
  std_msgs__msg__Int32 ticks_left;
  std_msgs__msg__Int32 ticks_right;
} robmovil_msgs__msg__EncoderTicks;

// Struct for a sequence of robmovil_msgs__msg__EncoderTicks.
typedef struct robmovil_msgs__msg__EncoderTicks__Sequence
{
  robmovil_msgs__msg__EncoderTicks * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robmovil_msgs__msg__EncoderTicks__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROBMOVIL_MSGS__MSG__DETAIL__ENCODER_TICKS__STRUCT_H_
