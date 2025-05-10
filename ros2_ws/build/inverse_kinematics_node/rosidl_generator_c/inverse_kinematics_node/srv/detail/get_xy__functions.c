// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from inverse_kinematics_node:srv/GetXY.idl
// generated code does not contain a copyright notice
#include "inverse_kinematics_node/srv/detail/get_xy__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

bool
inverse_kinematics_node__srv__GetXY_Request__init(inverse_kinematics_node__srv__GetXY_Request * msg)
{
  if (!msg) {
    return false;
  }
  // structure_needs_at_least_one_member
  return true;
}

void
inverse_kinematics_node__srv__GetXY_Request__fini(inverse_kinematics_node__srv__GetXY_Request * msg)
{
  if (!msg) {
    return;
  }
  // structure_needs_at_least_one_member
}

bool
inverse_kinematics_node__srv__GetXY_Request__are_equal(const inverse_kinematics_node__srv__GetXY_Request * lhs, const inverse_kinematics_node__srv__GetXY_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // structure_needs_at_least_one_member
  if (lhs->structure_needs_at_least_one_member != rhs->structure_needs_at_least_one_member) {
    return false;
  }
  return true;
}

bool
inverse_kinematics_node__srv__GetXY_Request__copy(
  const inverse_kinematics_node__srv__GetXY_Request * input,
  inverse_kinematics_node__srv__GetXY_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // structure_needs_at_least_one_member
  output->structure_needs_at_least_one_member = input->structure_needs_at_least_one_member;
  return true;
}

inverse_kinematics_node__srv__GetXY_Request *
inverse_kinematics_node__srv__GetXY_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  inverse_kinematics_node__srv__GetXY_Request * msg = (inverse_kinematics_node__srv__GetXY_Request *)allocator.allocate(sizeof(inverse_kinematics_node__srv__GetXY_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(inverse_kinematics_node__srv__GetXY_Request));
  bool success = inverse_kinematics_node__srv__GetXY_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
inverse_kinematics_node__srv__GetXY_Request__destroy(inverse_kinematics_node__srv__GetXY_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    inverse_kinematics_node__srv__GetXY_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
inverse_kinematics_node__srv__GetXY_Request__Sequence__init(inverse_kinematics_node__srv__GetXY_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  inverse_kinematics_node__srv__GetXY_Request * data = NULL;

  if (size) {
    data = (inverse_kinematics_node__srv__GetXY_Request *)allocator.zero_allocate(size, sizeof(inverse_kinematics_node__srv__GetXY_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = inverse_kinematics_node__srv__GetXY_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        inverse_kinematics_node__srv__GetXY_Request__fini(&data[i - 1]);
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
inverse_kinematics_node__srv__GetXY_Request__Sequence__fini(inverse_kinematics_node__srv__GetXY_Request__Sequence * array)
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
      inverse_kinematics_node__srv__GetXY_Request__fini(&array->data[i]);
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

inverse_kinematics_node__srv__GetXY_Request__Sequence *
inverse_kinematics_node__srv__GetXY_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  inverse_kinematics_node__srv__GetXY_Request__Sequence * array = (inverse_kinematics_node__srv__GetXY_Request__Sequence *)allocator.allocate(sizeof(inverse_kinematics_node__srv__GetXY_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = inverse_kinematics_node__srv__GetXY_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
inverse_kinematics_node__srv__GetXY_Request__Sequence__destroy(inverse_kinematics_node__srv__GetXY_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    inverse_kinematics_node__srv__GetXY_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
inverse_kinematics_node__srv__GetXY_Request__Sequence__are_equal(const inverse_kinematics_node__srv__GetXY_Request__Sequence * lhs, const inverse_kinematics_node__srv__GetXY_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!inverse_kinematics_node__srv__GetXY_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
inverse_kinematics_node__srv__GetXY_Request__Sequence__copy(
  const inverse_kinematics_node__srv__GetXY_Request__Sequence * input,
  inverse_kinematics_node__srv__GetXY_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(inverse_kinematics_node__srv__GetXY_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    inverse_kinematics_node__srv__GetXY_Request * data =
      (inverse_kinematics_node__srv__GetXY_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!inverse_kinematics_node__srv__GetXY_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          inverse_kinematics_node__srv__GetXY_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!inverse_kinematics_node__srv__GetXY_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


bool
inverse_kinematics_node__srv__GetXY_Response__init(inverse_kinematics_node__srv__GetXY_Response * msg)
{
  if (!msg) {
    return false;
  }
  // x
  // y
  return true;
}

void
inverse_kinematics_node__srv__GetXY_Response__fini(inverse_kinematics_node__srv__GetXY_Response * msg)
{
  if (!msg) {
    return;
  }
  // x
  // y
}

bool
inverse_kinematics_node__srv__GetXY_Response__are_equal(const inverse_kinematics_node__srv__GetXY_Response * lhs, const inverse_kinematics_node__srv__GetXY_Response * rhs)
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
inverse_kinematics_node__srv__GetXY_Response__copy(
  const inverse_kinematics_node__srv__GetXY_Response * input,
  inverse_kinematics_node__srv__GetXY_Response * output)
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

inverse_kinematics_node__srv__GetXY_Response *
inverse_kinematics_node__srv__GetXY_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  inverse_kinematics_node__srv__GetXY_Response * msg = (inverse_kinematics_node__srv__GetXY_Response *)allocator.allocate(sizeof(inverse_kinematics_node__srv__GetXY_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(inverse_kinematics_node__srv__GetXY_Response));
  bool success = inverse_kinematics_node__srv__GetXY_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
inverse_kinematics_node__srv__GetXY_Response__destroy(inverse_kinematics_node__srv__GetXY_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    inverse_kinematics_node__srv__GetXY_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
inverse_kinematics_node__srv__GetXY_Response__Sequence__init(inverse_kinematics_node__srv__GetXY_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  inverse_kinematics_node__srv__GetXY_Response * data = NULL;

  if (size) {
    data = (inverse_kinematics_node__srv__GetXY_Response *)allocator.zero_allocate(size, sizeof(inverse_kinematics_node__srv__GetXY_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = inverse_kinematics_node__srv__GetXY_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        inverse_kinematics_node__srv__GetXY_Response__fini(&data[i - 1]);
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
inverse_kinematics_node__srv__GetXY_Response__Sequence__fini(inverse_kinematics_node__srv__GetXY_Response__Sequence * array)
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
      inverse_kinematics_node__srv__GetXY_Response__fini(&array->data[i]);
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

inverse_kinematics_node__srv__GetXY_Response__Sequence *
inverse_kinematics_node__srv__GetXY_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  inverse_kinematics_node__srv__GetXY_Response__Sequence * array = (inverse_kinematics_node__srv__GetXY_Response__Sequence *)allocator.allocate(sizeof(inverse_kinematics_node__srv__GetXY_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = inverse_kinematics_node__srv__GetXY_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
inverse_kinematics_node__srv__GetXY_Response__Sequence__destroy(inverse_kinematics_node__srv__GetXY_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    inverse_kinematics_node__srv__GetXY_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
inverse_kinematics_node__srv__GetXY_Response__Sequence__are_equal(const inverse_kinematics_node__srv__GetXY_Response__Sequence * lhs, const inverse_kinematics_node__srv__GetXY_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!inverse_kinematics_node__srv__GetXY_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
inverse_kinematics_node__srv__GetXY_Response__Sequence__copy(
  const inverse_kinematics_node__srv__GetXY_Response__Sequence * input,
  inverse_kinematics_node__srv__GetXY_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(inverse_kinematics_node__srv__GetXY_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    inverse_kinematics_node__srv__GetXY_Response * data =
      (inverse_kinematics_node__srv__GetXY_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!inverse_kinematics_node__srv__GetXY_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          inverse_kinematics_node__srv__GetXY_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!inverse_kinematics_node__srv__GetXY_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
