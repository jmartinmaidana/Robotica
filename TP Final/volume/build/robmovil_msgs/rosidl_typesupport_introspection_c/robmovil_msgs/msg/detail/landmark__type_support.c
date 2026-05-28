// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from robmovil_msgs:msg/Landmark.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "robmovil_msgs/msg/detail/landmark__rosidl_typesupport_introspection_c.h"
#include "robmovil_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "robmovil_msgs/msg/detail/landmark__functions.h"
#include "robmovil_msgs/msg/detail/landmark__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void robmovil_msgs__msg__Landmark__rosidl_typesupport_introspection_c__Landmark_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  robmovil_msgs__msg__Landmark__init(message_memory);
}

void robmovil_msgs__msg__Landmark__rosidl_typesupport_introspection_c__Landmark_fini_function(void * message_memory)
{
  robmovil_msgs__msg__Landmark__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember robmovil_msgs__msg__Landmark__rosidl_typesupport_introspection_c__Landmark_message_member_array[2] = {
  {
    "range",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(robmovil_msgs__msg__Landmark, range),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "bearing",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(robmovil_msgs__msg__Landmark, bearing),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers robmovil_msgs__msg__Landmark__rosidl_typesupport_introspection_c__Landmark_message_members = {
  "robmovil_msgs__msg",  // message namespace
  "Landmark",  // message name
  2,  // number of fields
  sizeof(robmovil_msgs__msg__Landmark),
  robmovil_msgs__msg__Landmark__rosidl_typesupport_introspection_c__Landmark_message_member_array,  // message members
  robmovil_msgs__msg__Landmark__rosidl_typesupport_introspection_c__Landmark_init_function,  // function to initialize message memory (memory has to be allocated)
  robmovil_msgs__msg__Landmark__rosidl_typesupport_introspection_c__Landmark_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t robmovil_msgs__msg__Landmark__rosidl_typesupport_introspection_c__Landmark_message_type_support_handle = {
  0,
  &robmovil_msgs__msg__Landmark__rosidl_typesupport_introspection_c__Landmark_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_robmovil_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robmovil_msgs, msg, Landmark)() {
  if (!robmovil_msgs__msg__Landmark__rosidl_typesupport_introspection_c__Landmark_message_type_support_handle.typesupport_identifier) {
    robmovil_msgs__msg__Landmark__rosidl_typesupport_introspection_c__Landmark_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &robmovil_msgs__msg__Landmark__rosidl_typesupport_introspection_c__Landmark_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
