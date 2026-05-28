// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from robmovil_msgs:msg/MultiEncoderTicks.idl
// generated code does not contain a copyright notice

#ifndef ROBMOVIL_MSGS__MSG__DETAIL__MULTI_ENCODER_TICKS__STRUCT_H_
#define ROBMOVIL_MSGS__MSG__DETAIL__MULTI_ENCODER_TICKS__STRUCT_H_

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
// Member 'ticks'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in msg/MultiEncoderTicks in the package robmovil_msgs.
typedef struct robmovil_msgs__msg__MultiEncoderTicks
{
  std_msgs__msg__Header header;
  /// Each array element correspond to one joint encoder information.
  rosidl_runtime_c__int32__Sequence ticks;
} robmovil_msgs__msg__MultiEncoderTicks;

// Struct for a sequence of robmovil_msgs__msg__MultiEncoderTicks.
typedef struct robmovil_msgs__msg__MultiEncoderTicks__Sequence
{
  robmovil_msgs__msg__MultiEncoderTicks * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robmovil_msgs__msg__MultiEncoderTicks__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROBMOVIL_MSGS__MSG__DETAIL__MULTI_ENCODER_TICKS__STRUCT_H_
