// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from dual_dynamixel_node:srv/GetPositions.idl
// generated code does not contain a copyright notice
#include "dual_dynamixel_node/srv/detail/get_positions__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

// Include directives for member types
// Member `id`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
dual_dynamixel_node__srv__GetPositions_Request__init(dual_dynamixel_node__srv__GetPositions_Request * msg)
{
  if (!msg) {
    return false;
  }
  // id
  if (!rosidl_runtime_c__uint8__Sequence__init(&msg->id, 0)) {
    dual_dynamixel_node__srv__GetPositions_Request__fini(msg);
    return false;
  }
  return true;
}

void
dual_dynamixel_node__srv__GetPositions_Request__fini(dual_dynamixel_node__srv__GetPositions_Request * msg)
{
  if (!msg) {
    return;
  }
  // id
  rosidl_runtime_c__uint8__Sequence__fini(&msg->id);
}

bool
dual_dynamixel_node__srv__GetPositions_Request__are_equal(const dual_dynamixel_node__srv__GetPositions_Request * lhs, const dual_dynamixel_node__srv__GetPositions_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // id
  if (!rosidl_runtime_c__uint8__Sequence__are_equal(
      &(lhs->id), &(rhs->id)))
  {
    return false;
  }
  return true;
}

bool
dual_dynamixel_node__srv__GetPositions_Request__copy(
  const dual_dynamixel_node__srv__GetPositions_Request * input,
  dual_dynamixel_node__srv__GetPositions_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // id
  if (!rosidl_runtime_c__uint8__Sequence__copy(
      &(input->id), &(output->id)))
  {
    return false;
  }
  return true;
}

dual_dynamixel_node__srv__GetPositions_Request *
dual_dynamixel_node__srv__GetPositions_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dual_dynamixel_node__srv__GetPositions_Request * msg = (dual_dynamixel_node__srv__GetPositions_Request *)allocator.allocate(sizeof(dual_dynamixel_node__srv__GetPositions_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(dual_dynamixel_node__srv__GetPositions_Request));
  bool success = dual_dynamixel_node__srv__GetPositions_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
dual_dynamixel_node__srv__GetPositions_Request__destroy(dual_dynamixel_node__srv__GetPositions_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    dual_dynamixel_node__srv__GetPositions_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
dual_dynamixel_node__srv__GetPositions_Request__Sequence__init(dual_dynamixel_node__srv__GetPositions_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dual_dynamixel_node__srv__GetPositions_Request * data = NULL;

  if (size) {
    data = (dual_dynamixel_node__srv__GetPositions_Request *)allocator.zero_allocate(size, sizeof(dual_dynamixel_node__srv__GetPositions_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = dual_dynamixel_node__srv__GetPositions_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        dual_dynamixel_node__srv__GetPositions_Request__fini(&data[i - 1]);
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
dual_dynamixel_node__srv__GetPositions_Request__Sequence__fini(dual_dynamixel_node__srv__GetPositions_Request__Sequence * array)
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
      dual_dynamixel_node__srv__GetPositions_Request__fini(&array->data[i]);
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

dual_dynamixel_node__srv__GetPositions_Request__Sequence *
dual_dynamixel_node__srv__GetPositions_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dual_dynamixel_node__srv__GetPositions_Request__Sequence * array = (dual_dynamixel_node__srv__GetPositions_Request__Sequence *)allocator.allocate(sizeof(dual_dynamixel_node__srv__GetPositions_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = dual_dynamixel_node__srv__GetPositions_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
dual_dynamixel_node__srv__GetPositions_Request__Sequence__destroy(dual_dynamixel_node__srv__GetPositions_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    dual_dynamixel_node__srv__GetPositions_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
dual_dynamixel_node__srv__GetPositions_Request__Sequence__are_equal(const dual_dynamixel_node__srv__GetPositions_Request__Sequence * lhs, const dual_dynamixel_node__srv__GetPositions_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!dual_dynamixel_node__srv__GetPositions_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
dual_dynamixel_node__srv__GetPositions_Request__Sequence__copy(
  const dual_dynamixel_node__srv__GetPositions_Request__Sequence * input,
  dual_dynamixel_node__srv__GetPositions_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(dual_dynamixel_node__srv__GetPositions_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    dual_dynamixel_node__srv__GetPositions_Request * data =
      (dual_dynamixel_node__srv__GetPositions_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!dual_dynamixel_node__srv__GetPositions_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          dual_dynamixel_node__srv__GetPositions_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!dual_dynamixel_node__srv__GetPositions_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `position`
// already included above
// #include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
dual_dynamixel_node__srv__GetPositions_Response__init(dual_dynamixel_node__srv__GetPositions_Response * msg)
{
  if (!msg) {
    return false;
  }
  // position
  if (!rosidl_runtime_c__int32__Sequence__init(&msg->position, 0)) {
    dual_dynamixel_node__srv__GetPositions_Response__fini(msg);
    return false;
  }
  return true;
}

void
dual_dynamixel_node__srv__GetPositions_Response__fini(dual_dynamixel_node__srv__GetPositions_Response * msg)
{
  if (!msg) {
    return;
  }
  // position
  rosidl_runtime_c__int32__Sequence__fini(&msg->position);
}

bool
dual_dynamixel_node__srv__GetPositions_Response__are_equal(const dual_dynamixel_node__srv__GetPositions_Response * lhs, const dual_dynamixel_node__srv__GetPositions_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // position
  if (!rosidl_runtime_c__int32__Sequence__are_equal(
      &(lhs->position), &(rhs->position)))
  {
    return false;
  }
  return true;
}

bool
dual_dynamixel_node__srv__GetPositions_Response__copy(
  const dual_dynamixel_node__srv__GetPositions_Response * input,
  dual_dynamixel_node__srv__GetPositions_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // position
  if (!rosidl_runtime_c__int32__Sequence__copy(
      &(input->position), &(output->position)))
  {
    return false;
  }
  return true;
}

dual_dynamixel_node__srv__GetPositions_Response *
dual_dynamixel_node__srv__GetPositions_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dual_dynamixel_node__srv__GetPositions_Response * msg = (dual_dynamixel_node__srv__GetPositions_Response *)allocator.allocate(sizeof(dual_dynamixel_node__srv__GetPositions_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(dual_dynamixel_node__srv__GetPositions_Response));
  bool success = dual_dynamixel_node__srv__GetPositions_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
dual_dynamixel_node__srv__GetPositions_Response__destroy(dual_dynamixel_node__srv__GetPositions_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    dual_dynamixel_node__srv__GetPositions_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
dual_dynamixel_node__srv__GetPositions_Response__Sequence__init(dual_dynamixel_node__srv__GetPositions_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dual_dynamixel_node__srv__GetPositions_Response * data = NULL;

  if (size) {
    data = (dual_dynamixel_node__srv__GetPositions_Response *)allocator.zero_allocate(size, sizeof(dual_dynamixel_node__srv__GetPositions_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = dual_dynamixel_node__srv__GetPositions_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        dual_dynamixel_node__srv__GetPositions_Response__fini(&data[i - 1]);
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
dual_dynamixel_node__srv__GetPositions_Response__Sequence__fini(dual_dynamixel_node__srv__GetPositions_Response__Sequence * array)
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
      dual_dynamixel_node__srv__GetPositions_Response__fini(&array->data[i]);
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

dual_dynamixel_node__srv__GetPositions_Response__Sequence *
dual_dynamixel_node__srv__GetPositions_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dual_dynamixel_node__srv__GetPositions_Response__Sequence * array = (dual_dynamixel_node__srv__GetPositions_Response__Sequence *)allocator.allocate(sizeof(dual_dynamixel_node__srv__GetPositions_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = dual_dynamixel_node__srv__GetPositions_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
dual_dynamixel_node__srv__GetPositions_Response__Sequence__destroy(dual_dynamixel_node__srv__GetPositions_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    dual_dynamixel_node__srv__GetPositions_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
dual_dynamixel_node__srv__GetPositions_Response__Sequence__are_equal(const dual_dynamixel_node__srv__GetPositions_Response__Sequence * lhs, const dual_dynamixel_node__srv__GetPositions_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!dual_dynamixel_node__srv__GetPositions_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
dual_dynamixel_node__srv__GetPositions_Response__Sequence__copy(
  const dual_dynamixel_node__srv__GetPositions_Response__Sequence * input,
  dual_dynamixel_node__srv__GetPositions_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(dual_dynamixel_node__srv__GetPositions_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    dual_dynamixel_node__srv__GetPositions_Response * data =
      (dual_dynamixel_node__srv__GetPositions_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!dual_dynamixel_node__srv__GetPositions_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          dual_dynamixel_node__srv__GetPositions_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!dual_dynamixel_node__srv__GetPositions_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
