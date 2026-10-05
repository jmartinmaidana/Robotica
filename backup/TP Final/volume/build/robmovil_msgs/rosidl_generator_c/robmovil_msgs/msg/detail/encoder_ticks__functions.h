// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from robmovil_msgs:msg/EncoderTicks.idl
// generated code does not contain a copyright notice

#ifndef ROBMOVIL_MSGS__MSG__DETAIL__ENCODER_TICKS__FUNCTIONS_H_
#define ROBMOVIL_MSGS__MSG__DETAIL__ENCODER_TICKS__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "robmovil_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "robmovil_msgs/msg/detail/encoder_ticks__struct.h"

/// Initialize msg/EncoderTicks message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * robmovil_msgs__msg__EncoderTicks
 * )) before or use
 * robmovil_msgs__msg__EncoderTicks__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_robmovil_msgs
bool
robmovil_msgs__msg__EncoderTicks__init(robmovil_msgs__msg__EncoderTicks * msg);

/// Finalize msg/EncoderTicks message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_robmovil_msgs
void
robmovil_msgs__msg__EncoderTicks__fini(robmovil_msgs__msg__EncoderTicks * msg);

/// Create msg/EncoderTicks message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * robmovil_msgs__msg__EncoderTicks__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_robmovil_msgs
robmovil_msgs__msg__EncoderTicks *
robmovil_msgs__msg__EncoderTicks__create();

/// Destroy msg/EncoderTicks message.
/**
 * It calls
 * robmovil_msgs__msg__EncoderTicks__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_robmovil_msgs
void
robmovil_msgs__msg__EncoderTicks__destroy(robmovil_msgs__msg__EncoderTicks * msg);

/// Check for msg/EncoderTicks message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_robmovil_msgs
bool
robmovil_msgs__msg__EncoderTicks__are_equal(const robmovil_msgs__msg__EncoderTicks * lhs, const robmovil_msgs__msg__EncoderTicks * rhs);

/// Copy a msg/EncoderTicks message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_robmovil_msgs
bool
robmovil_msgs__msg__EncoderTicks__copy(
  const robmovil_msgs__msg__EncoderTicks * input,
  robmovil_msgs__msg__EncoderTicks * output);

/// Initialize array of msg/EncoderTicks messages.
/**
 * It allocates the memory for the number of elements and calls
 * robmovil_msgs__msg__EncoderTicks__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_robmovil_msgs
bool
robmovil_msgs__msg__EncoderTicks__Sequence__init(robmovil_msgs__msg__EncoderTicks__Sequence * array, size_t size);

/// Finalize array of msg/EncoderTicks messages.
/**
 * It calls
 * robmovil_msgs__msg__EncoderTicks__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_robmovil_msgs
void
robmovil_msgs__msg__EncoderTicks__Sequence__fini(robmovil_msgs__msg__EncoderTicks__Sequence * array);

/// Create array of msg/EncoderTicks messages.
/**
 * It allocates the memory for the array and calls
 * robmovil_msgs__msg__EncoderTicks__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_robmovil_msgs
robmovil_msgs__msg__EncoderTicks__Sequence *
robmovil_msgs__msg__EncoderTicks__Sequence__create(size_t size);

/// Destroy array of msg/EncoderTicks messages.
/**
 * It calls
 * robmovil_msgs__msg__EncoderTicks__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_robmovil_msgs
void
robmovil_msgs__msg__EncoderTicks__Sequence__destroy(robmovil_msgs__msg__EncoderTicks__Sequence * array);

/// Check for msg/EncoderTicks message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_robmovil_msgs
bool
robmovil_msgs__msg__EncoderTicks__Sequence__are_equal(const robmovil_msgs__msg__EncoderTicks__Sequence * lhs, const robmovil_msgs__msg__EncoderTicks__Sequence * rhs);

/// Copy an array of msg/EncoderTicks messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_robmovil_msgs
bool
robmovil_msgs__msg__EncoderTicks__Sequence__copy(
  const robmovil_msgs__msg__EncoderTicks__Sequence * input,
  robmovil_msgs__msg__EncoderTicks__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // ROBMOVIL_MSGS__MSG__DETAIL__ENCODER_TICKS__FUNCTIONS_H_
