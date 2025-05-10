// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from joint_positions_node:msg/SetJointDegrees.idl
// generated code does not contain a copyright notice
#include "joint_positions_node/msg/detail/set_joint_degrees__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
joint_positions_node__msg__SetJointDegrees__init(joint_positions_node__msg__SetJointDegrees * msg)
{
  if (!msg) {
    return false;
  }
  // degrees
  return true;
}

void
joint_positions_node__msg__SetJointDegrees__fini(joint_positions_node__msg__SetJointDegrees * msg)
{
  if (!msg) {
    return;
  }
  // degrees
}

bool
joint_positions_node__msg__SetJointDegrees__are_equal(const joint_positions_node__msg__SetJointDegrees * lhs, const joint_positions_node__msg__SetJointDegrees * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // degrees
  for (size_t i = 0; i < 2; ++i) {
    if (lhs->degrees[i] != rhs->degrees[i]) {
      return false;
    }
  }
  return true;
}

bool
joint_positions_node__msg__SetJointDegrees__copy(
  const joint_positions_node__msg__SetJointDegrees * input,
  joint_positions_node__msg__SetJointDegrees * output)
{
  if (!input || !output) {
    return false;
  }
  // degrees
  for (size_t i = 0; i < 2; ++i) {
    output->degrees[i] = input->degrees[i];
  }
  return true;
}

joint_positions_node__msg__SetJointDegrees *
joint_positions_node__msg__SetJointDegrees__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  joint_positions_node__msg__SetJointDegrees * msg = (joint_positions_node__msg__SetJointDegrees *)allocator.allocate(sizeof(joint_positions_node__msg__SetJointDegrees), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(joint_positions_node__msg__SetJointDegrees));
  bool success = joint_positions_node__msg__SetJointDegrees__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
joint_positions_node__msg__SetJointDegrees__destroy(joint_positions_node__msg__SetJointDegrees * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    joint_positions_node__msg__SetJointDegrees__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
joint_positions_node__msg__SetJointDegrees__Sequence__init(joint_positions_node__msg__SetJointDegrees__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  joint_positions_node__msg__SetJointDegrees * data = NULL;

  if (size) {
    data = (joint_positions_node__msg__SetJointDegrees *)allocator.zero_allocate(size, sizeof(joint_positions_node__msg__SetJointDegrees), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = joint_positions_node__msg__SetJointDegrees__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        joint_positions_node__msg__SetJointDegrees__fini(&data[i - 1]);
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
joint_positions_node__msg__SetJointDegrees__Sequence__fini(joint_positions_node__msg__SetJointDegrees__Sequence * array)
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
      joint_positions_node__msg__SetJointDegrees__fini(&array->data[i]);
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

joint_positions_node__msg__SetJointDegrees__Sequence *
joint_positions_node__msg__SetJointDegrees__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  joint_positions_node__msg__SetJointDegrees__Sequence * array = (joint_positions_node__msg__SetJointDegrees__Sequence *)allocator.allocate(sizeof(joint_positions_node__msg__SetJointDegrees__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = joint_positions_node__msg__SetJointDegrees__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
joint_positions_node__msg__SetJointDegrees__Sequence__destroy(joint_positions_node__msg__SetJointDegrees__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    joint_positions_node__msg__SetJointDegrees__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
joint_positions_node__msg__SetJointDegrees__Sequence__are_equal(const joint_positions_node__msg__SetJointDegrees__Sequence * lhs, const joint_positions_node__msg__SetJointDegrees__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!joint_positions_node__msg__SetJointDegrees__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
joint_positions_node__msg__SetJointDegrees__Sequence__copy(
  const joint_positions_node__msg__SetJointDegrees__Sequence * input,
  joint_positions_node__msg__SetJointDegrees__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(joint_positions_node__msg__SetJointDegrees);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    joint_positions_node__msg__SetJointDegrees * data =
      (joint_positions_node__msg__SetJointDegrees *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!joint_positions_node__msg__SetJointDegrees__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          joint_positions_node__msg__SetJointDegrees__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!joint_positions_node__msg__SetJointDegrees__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
