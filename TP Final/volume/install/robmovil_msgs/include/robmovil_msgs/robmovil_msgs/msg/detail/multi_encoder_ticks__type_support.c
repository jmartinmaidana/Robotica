// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from robmovil_msgs:msg/MultiEncoderTicks.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "robmovil_msgs/msg/detail/multi_encoder_ticks__rosidl_typesupport_introspection_c.h"
#include "robmovil_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "robmovil_msgs/msg/detail/multi_encoder_ticks__functions.h"
#include "robmovil_msgs/msg/detail/multi_encoder_ticks__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `ticks`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void robmovil_msgs__msg__MultiEncoderTicks__rosidl_typesupport_introspection_c__MultiEncoderTicks_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  robmovil_msgs__msg__MultiEncoderTicks__init(message_memory);
}

void robmovil_msgs__msg__MultiEncoderTicks__rosidl_typesupport_introspection_c__MultiEncoderTicks_fini_function(void * message_memory)
{
  robmovil_msgs__msg__MultiEncoderTicks__fini(message_memory);
}

size_t robmovil_msgs__msg__MultiEncoderTicks__rosidl_typesupport_introspection_c__size_function__MultiEncoderTicks__ticks(
  const void * untyped_member)
{
  const rosidl_runtime_c__int32__Sequence * member =
    (const rosidl_runtime_c__int32__Sequence *)(untyped_member);
  return member->size;
}

const void * robmovil_msgs__msg__MultiEncoderTicks__rosidl_typesupport_introspection_c__get_const_function__MultiEncoderTicks__ticks(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__int32__Sequence * member =
    (const rosidl_runtime_c__int32__Sequence *)(untyped_member);
  return &member->data[index];
}

void * robmovil_msgs__msg__MultiEncoderTicks__rosidl_typesupport_introspection_c__get_function__MultiEncoderTicks__ticks(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__int32__Sequence * member =
    (rosidl_runtime_c__int32__Sequence *)(untyped_member);
  return &member->data[index];
}

void robmovil_msgs__msg__MultiEncoderTicks__rosidl_typesupport_introspection_c__fetch_function__MultiEncoderTicks__ticks(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const int32_t * item =
    ((const int32_t *)
    robmovil_msgs__msg__MultiEncoderTicks__rosidl_typesupport_introspection_c__get_const_function__MultiEncoderTicks__ticks(untyped_member, index));
  int32_t * value =
    (int32_t *)(untyped_value);
  *value = *item;
}

void robmovil_msgs__msg__MultiEncoderTicks__rosidl_typesupport_introspection_c__assign_function__MultiEncoderTicks__ticks(
  void * untyped_member, size_t index, const void * untyped_value)
{
  int32_t * item =
    ((int32_t *)
    robmovil_msgs__msg__MultiEncoderTicks__rosidl_typesupport_introspection_c__get_function__MultiEncoderTicks__ticks(untyped_member, index));
  const int32_t * value =
    (const int32_t *)(untyped_value);
  *item = *value;
}

bool robmovil_msgs__msg__MultiEncoderTicks__rosidl_typesupport_introspection_c__resize_function__MultiEncoderTicks__ticks(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__int32__Sequence * member =
    (rosidl_runtime_c__int32__Sequence *)(untyped_member);
  rosidl_runtime_c__int32__Sequence__fini(member);
  return rosidl_runtime_c__int32__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember robmovil_msgs__msg__MultiEncoderTicks__rosidl_typesupport_introspection_c__MultiEncoderTicks_message_member_array[2] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(robmovil_msgs__msg__MultiEncoderTicks, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "ticks",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(robmovil_msgs__msg__MultiEncoderTicks, ticks),  // bytes offset in struct
    NULL,  // default value
    robmovil_msgs__msg__MultiEncoderTicks__rosidl_typesupport_introspection_c__size_function__MultiEncoderTicks__ticks,  // size() function pointer
    robmovil_msgs__msg__MultiEncoderTicks__rosidl_typesupport_introspection_c__get_const_function__MultiEncoderTicks__ticks,  // get_const(index) function pointer
    robmovil_msgs__msg__MultiEncoderTicks__rosidl_typesupport_introspection_c__get_function__MultiEncoderTicks__ticks,  // get(index) function pointer
    robmovil_msgs__msg__MultiEncoderTicks__rosidl_typesupport_introspection_c__fetch_function__MultiEncoderTicks__ticks,  // fetch(index, &value) function pointer
    robmovil_msgs__msg__MultiEncoderTicks__rosidl_typesupport_introspection_c__assign_function__MultiEncoderTicks__ticks,  // assign(index, value) function pointer
    robmovil_msgs__msg__MultiEncoderTicks__rosidl_typesupport_introspection_c__resize_function__MultiEncoderTicks__ticks  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers robmovil_msgs__msg__MultiEncoderTicks__rosidl_typesupport_introspection_c__MultiEncoderTicks_message_members = {
  "robmovil_msgs__msg",  // message namespace
  "MultiEncoderTicks",  // message name
  2,  // number of fields
  sizeof(robmovil_msgs__msg__MultiEncoderTicks),
  robmovil_msgs__msg__MultiEncoderTicks__rosidl_typesupport_introspection_c__MultiEncoderTicks_message_member_array,  // message members
  robmovil_msgs__msg__MultiEncoderTicks__rosidl_typesupport_introspection_c__MultiEncoderTicks_init_function,  // function to initialize message memory (memory has to be allocated)
  robmovil_msgs__msg__MultiEncoderTicks__rosidl_typesupport_introspection_c__MultiEncoderTicks_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t robmovil_msgs__msg__MultiEncoderTicks__rosidl_typesupport_introspection_c__MultiEncoderTicks_message_type_support_handle = {
  0,
  &robmovil_msgs__msg__MultiEncoderTicks__rosidl_typesupport_introspection_c__MultiEncoderTicks_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_robmovil_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robmovil_msgs, msg, MultiEncoderTicks)() {
  robmovil_msgs__msg__MultiEncoderTicks__rosidl_typesupport_introspection_c__MultiEncoderTicks_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  if (!robmovil_msgs__msg__MultiEncoderTicks__rosidl_typesupport_introspection_c__MultiEncoderTicks_message_type_support_handle.typesupport_identifier) {
    robmovil_msgs__msg__MultiEncoderTicks__rosidl_typesupport_introspection_c__MultiEncoderTicks_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &robmovil_msgs__msg__MultiEncoderTicks__rosidl_typesupport_introspection_c__MultiEncoderTicks_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
