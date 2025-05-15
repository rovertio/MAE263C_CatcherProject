// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from inverse_kinematics_node:msg/SetXY.idl
// generated code does not contain a copyright notice
#include "inverse_kinematics_node/msg/detail/set_xy__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
inverse_kinematics_node__msg__SetXY__init(inverse_kinematics_node__msg__SetXY * msg)
{
  if (!msg) {
    return false;
  }
  // x
  // y
  return true;
}

void
inverse_kinematics_node__msg__SetXY__fini(inverse_kinematics_node__msg__SetXY * msg)
{
  if (!msg) {
    return;
  }
  // x
  // y
}

bool
inverse_kinematics_node__msg__SetXY__are_equal(const inverse_kinematics_node__msg__SetXY * lhs, const inverse_kinematics_node__msg__SetXY * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // x
  if (lhs->x != rhs->x) {
    return false;
  }
  // y
  if (lhs->y != rhs->y) {
    return false;
  }
  return true;
}

bool
inverse_kinematics_node__msg__SetXY__copy(
  const inverse_kinematics_node__msg__SetXY * input,
  inverse_kinematics_node__msg__SetXY * output)
{
  if (!input || !output) {
    return false;
  }
  // x
  output->x = input->x;
  // y
  output->y = input->y;
  return true;
}

inverse_kinematics_node__msg__SetXY *
inverse_kinematics_node__msg__SetXY__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  inverse_kinematics_node__msg__SetXY * msg = (inverse_kinematics_node__msg__SetXY *)allocator.allocate(sizeof(inverse_kinematics_node__msg__SetXY), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(inverse_kinematics_node__msg__SetXY));
  bool success = inverse_kinematics_node__msg__SetXY__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
inverse_kinematics_node__msg__SetXY__destroy(inverse_kinematics_node__msg__SetXY * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    inverse_kinematics_node__msg__SetXY__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
inverse_kinematics_node__msg__SetXY__Sequence__init(inverse_kinematics_node__msg__SetXY__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  inverse_kinematics_node__msg__SetXY * data = NULL;

  if (size) {
    data = (inverse_kinematics_node__msg__SetXY *)allocator.zero_allocate(size, sizeof(inverse_kinematics_node__msg__SetXY), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = inverse_kinematics_node__msg__SetXY__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        inverse_kinematics_node__msg__SetXY__fini(&data[i - 1]);
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
inverse_kinematics_node__msg__SetXY__Sequence__fini(inverse_kinematics_node__msg__SetXY__Sequence * array)
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
      inverse_kinematics_node__msg__SetXY__fini(&array->data[i]);
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

inverse_kinematics_node__msg__SetXY__Sequence *
inverse_kinematics_node__msg__SetXY__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  inverse_kinematics_node__msg__SetXY__Sequence * array = (inverse_kinematics_node__msg__SetXY__Sequence *)allocator.allocate(sizeof(inverse_kinematics_node__msg__SetXY__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = inverse_kinematics_node__msg__SetXY__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
inverse_kinematics_node__msg__SetXY__Sequence__destroy(inverse_kinematics_node__msg__SetXY__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    inverse_kinematics_node__msg__SetXY__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
inverse_kinematics_node__msg__SetXY__Sequence__are_equal(const inverse_kinematics_node__msg__SetXY__Sequence * lhs, const inverse_kinematics_node__msg__SetXY__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!inverse_kinematics_node__msg__SetXY__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
inverse_kinematics_node__msg__SetXY__Sequence__copy(
  const inverse_kinematics_node__msg__SetXY__Sequence * input,
  inverse_kinematics_node__msg__SetXY__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(inverse_kinematics_node__msg__SetXY);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    inverse_kinematics_node__msg__SetXY * data =
      (inverse_kinematics_node__msg__SetXY *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!inverse_kinematics_node__msg__SetXY__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          inverse_kinematics_node__msg__SetXY__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!inverse_kinematics_node__msg__SetXY__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
