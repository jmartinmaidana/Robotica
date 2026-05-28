// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from robmovil_msgs:msg/Landmark.idl
// generated code does not contain a copyright notice

#ifndef ROBMOVIL_MSGS__MSG__DETAIL__LANDMARK__STRUCT_H_
#define ROBMOVIL_MSGS__MSG__DETAIL__LANDMARK__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/Landmark in the package robmovil_msgs.
typedef struct robmovil_msgs__msg__Landmark
{
  float range;
  float bearing;
} robmovil_msgs__msg__Landmark;

// Struct for a sequence of robmovil_msgs__msg__Landmark.
typedef struct robmovil_msgs__msg__Landmark__Sequence
{
  robmovil_msgs__msg__Landmark * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robmovil_msgs__msg__Landmark__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROBMOVIL_MSGS__MSG__DETAIL__LANDMARK__STRUCT_H_
