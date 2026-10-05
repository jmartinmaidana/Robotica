// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from robmovil_msgs:msg/Trajectory.idl
// generated code does not contain a copyright notice

#ifndef ROBMOVIL_MSGS__MSG__DETAIL__TRAJECTORY__STRUCT_H_
#define ROBMOVIL_MSGS__MSG__DETAIL__TRAJECTORY__STRUCT_H_

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
// Member 'points'
#include "robmovil_msgs/msg/detail/trajectory_point__struct.h"

/// Struct defined in msg/Trajectory in the package robmovil_msgs.
/**
  * The header is used to specify the coordinate frame
  * and the reference time for the trajectory durations.
 */
typedef struct robmovil_msgs__msg__Trajectory
{
  std_msgs__msg__Header header;
  /// List of Trajectory waypoints.
  robmovil_msgs__msg__TrajectoryPoint__Sequence points;
} robmovil_msgs__msg__Trajectory;

// Struct for a sequence of robmovil_msgs__msg__Trajectory.
typedef struct robmovil_msgs__msg__Trajectory__Sequence
{
  robmovil_msgs__msg__Trajectory * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robmovil_msgs__msg__Trajectory__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROBMOVIL_MSGS__MSG__DETAIL__TRAJECTORY__STRUCT_H_
