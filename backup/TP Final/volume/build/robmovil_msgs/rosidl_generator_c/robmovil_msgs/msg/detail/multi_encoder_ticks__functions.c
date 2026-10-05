// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from robmovil_msgs:msg/MultiEncoderTicks.idl
// generated code does not contain a copyright notice
#include "robmovil_msgs/msg/detail/multi_encoder_ticks__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `ticks`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
robmovil_msgs__msg__MultiEncoderTicks__init(robmovil_msgs__msg__MultiEncoderTicks * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    robmovil_msgs__msg__MultiEncoderTicks__fini(msg);
    return false;
  }
  // ticks
  if (!rosidl_runtime_c__int32__Sequence__init(&msg->ticks, 0)) {
    robmovil_msgs__msg__MultiEncoderTicks__fini(msg);
    return false;
  }
  return true;
}

void
robmovil_msgs__msg__MultiEncoderTicks__fini(robmovil_msgs__msg__MultiEncoderTicks * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // ticks
  rosidl_runtime_c__int32__Sequence__fini(&msg->ticks);
}

bool
robmovil_msgs__msg__MultiEncoderTicks__are_equal(const robmovil_msgs__msg__MultiEncoderTicks * lhs, const robmovil_msgs__msg__MultiEncoderTicks * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // ticks
  if (!rosidl_runtime_c__int32__Sequence__are_equal(
      &(lhs->ticks), &(rhs->ticks)))
  {
    return false;
  }
  return true;
}

bool
robmovil_msgs__msg__MultiEncoderTicks__copy(
  const robmovil_msgs__msg__MultiEncoderTicks * input,
  robmovil_msgs__msg__MultiEncoderTicks * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // ticks
  if (!rosidl_runtime_c__int32__Sequence__copy(
      &(input->ticks), &(output->ticks)))
  {
    return false;
  }
  return true;
}

robmovil_msgs__msg__MultiEncoderTicks *
robmovil_msgs__msg__MultiEncoderTicks__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robmovil_msgs__msg__MultiEncoderTicks * msg = (robmovil_msgs__msg__MultiEncoderTicks *)allocator.allocate(sizeof(robmovil_msgs__msg__MultiEncoderTicks), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(robmovil_msgs__msg__MultiEncoderTicks));
  bool success = robmovil_msgs__msg__MultiEncoderTicks__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
robmovil_msgs__msg__MultiEncoderTicks__destroy(robmovil_msgs__msg__MultiEncoderTicks * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    robmovil_msgs__msg__MultiEncoderTicks__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
robmovil_msgs__msg__MultiEncoderTicks__Sequence__init(robmovil_msgs__msg__MultiEncoderTicks__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robmovil_msgs__msg__MultiEncoderTicks * data = NULL;

  if (size) {
    data = (robmovil_msgs__msg__MultiEncoderTicks *)allocator.zero_allocate(size, sizeof(robmovil_msgs__msg__MultiEncoderTicks), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = robmovil_msgs__msg__MultiEncoderTicks__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        robmovil_msgs__msg__MultiEncoderTicks__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
robmovil_msgs__msg__MultiEncoderTicks__Sequence__fini(robmovil_msgs__msg__MultiEncoderTicks__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      robmovil_msgs__msg__MultiEncoderTicks__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

robmovil_msgs__msg__MultiEncoderTicks__Sequence *
robmovil_msgs__msg__MultiEncoderTicks__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robmovil_msgs__msg__MultiEncoderTicks__Sequence * array = (robmovil_msgs__msg__MultiEncoderTicks__Sequence *)allocator.allocate(sizeof(robmovil_msgs__msg__MultiEncoderTicks__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = robmovil_msgs__msg__MultiEncoderTicks__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
robmovil_msgs__msg__MultiEncoderTicks__Sequence__destroy(robmovil_msgs__msg__MultiEncoderTicks__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    robmovil_msgs__msg__MultiEncoderTicks__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
robmovil_msgs__msg__MultiEncoderTicks__Sequence__are_equal(const robmovil_msgs__msg__MultiEncoderTicks__Sequence * lhs, const robmovil_msgs__msg__MultiEncoderTicks__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!robmovil_msgs__msg__MultiEncoderTicks__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
robmovil_msgs__msg__MultiEncoderTicks__Sequence__copy(
  const robmovil_msgs__msg__MultiEncoderTicks__Sequence * input,
  robmovil_msgs__msg__MultiEncoderTicks__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(robmovil_msgs__msg__MultiEncoderTicks);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    robmovil_msgs__msg__MultiEncoderTicks * data =
      (robmovil_msgs__msg__MultiEncoderTicks *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!robmovil_msgs__msg__MultiEncoderTicks__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          robmovil_msgs__msg__MultiEncoderTicks__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!robmovil_msgs__msg__MultiEncoderTicks__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
