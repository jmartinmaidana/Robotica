// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from robmovil_msgs:msg/LandmarkArray.idl
// generated code does not contain a copyright notice

#ifndef ROBMOVIL_MSGS__MSG__DETAIL__LANDMARK_ARRAY__STRUCT_H_
#define ROBMOVIL_MSGS__MSG__DETAIL__LANDMARK_ARRAY__STRUCT_H_

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
// Member 'landmarks'
#include "robmovil_msgs/msg/detail/landmark__struct.h"

/// Struct defined in msg/LandmarkArray in the package robmovil_msgs.
typedef struct robmovil_msgs__msg__LandmarkArray
{
  std_msgs__msg__Header header;
  robmovil_msgs__msg__Landmark__Sequence landmarks;
} robmovil_msgs__msg__LandmarkArray;

// Struct for a sequence of robmovil_msgs__msg__LandmarkArray.
typedef struct robmovil_msgs__msg__LandmarkArray__Sequence
{
  robmovil_msgs__msg__LandmarkArray * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robmovil_msgs__msg__LandmarkArray__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROBMOVIL_MSGS__MSG__DETAIL__LANDMARK_ARRAY__STRUCT_H_
