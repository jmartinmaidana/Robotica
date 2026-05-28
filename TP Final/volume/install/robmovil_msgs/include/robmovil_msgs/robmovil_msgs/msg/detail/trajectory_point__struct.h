// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from robmovil_msgs:msg/TrajectoryPoint.idl
// generated code does not contain a copyright notice

#ifndef ROBMOVIL_MSGS__MSG__DETAIL__TRAJECTORY_POINT__STRUCT_H_
#define ROBMOVIL_MSGS__MSG__DETAIL__TRAJECTORY_POINT__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'transform'
#include "geometry_msgs/msg/detail/transform__struct.h"
// Member 'velocity'
// Member 'acceleration'
#include "geometry_msgs/msg/detail/twist__struct.h"
// Member 'time_from_start'
#include "builtin_interfaces/msg/detail/duration__struct.h"

/// Struct defined in msg/TrajectoryPoint in the package robmovil_msgs.
/**
  * Each multi-dof joint can specify a transform (up to 6 DOF)
 */
typedef struct robmovil_msgs__msg__TrajectoryPoint
{
  geometry_msgs__msg__Transform transform;
  /// There can be a velocity specified for the origin of the joint
  geometry_msgs__msg__Twist velocity;
  /// There can be an acceleration specified for the origin of the joint
  geometry_msgs__msg__Twist acceleration;
  builtin_interfaces__msg__Duration time_from_start;
} robmovil_msgs__msg__TrajectoryPoint;

// Struct for a sequence of robmovil_msgs__msg__TrajectoryPoint.
typedef struct robmovil_msgs__msg__TrajectoryPoint__Sequence
{
  robmovil_msgs__msg__TrajectoryPoint * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robmovil_msgs__msg__TrajectoryPoint__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROBMOVIL_MSGS__MSG__DETAIL__TRAJECTORY_POINT__STRUCT_H_
