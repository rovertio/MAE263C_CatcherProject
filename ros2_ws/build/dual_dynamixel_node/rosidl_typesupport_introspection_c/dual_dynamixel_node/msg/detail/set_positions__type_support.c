// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from dual_dynamixel_node:msg/SetPositions.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "dual_dynamixel_node/msg/detail/set_positions__rosidl_typesupport_introspection_c.h"
#include "dual_dynamixel_node/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "dual_dynamixel_node/msg/detail/set_positions__functions.h"
#include "dual_dynamixel_node/msg/detail/set_positions__struct.h"


// Include directives for member types
// Member `id`
// Member `position`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void dual_dynamixel_node__msg__SetPositions__rosidl_typesupport_introspection_c__SetPositions_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  dual_dynamixel_node__msg__SetPositions__init(message_memory);
}

void dual_dynamixel_node__msg__SetPositions__rosidl_typesupport_introspection_c__SetPositions_fini_function(void * message_memory)
{
  dual_dynamixel_node__msg__SetPositions__fini(message_memory);
}

size_t dual_dynamixel_node__msg__SetPositions__rosidl_typesupport_introspection_c__size_function__SetPositions__id(
  const void * untyped_member)
{
  const rosidl_runtime_c__uint8__Sequence * member =
    (const rosidl_runtime_c__uint8__Sequence *)(untyped_member);
  return member->size;
}

const void * dual_dynamixel_node__msg__SetPositions__rosidl_typesupport_introspection_c__get_const_function__SetPositions__id(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__uint8__Sequence * member =
    (const rosidl_runtime_c__uint8__Sequence *)(untyped_member);
  return &member->data[index];
}

void * dual_dynamixel_node__msg__SetPositions__rosidl_typesupport_introspection_c__get_function__SetPositions__id(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__uint8__Sequence * member =
    (rosidl_runtime_c__uint8__Sequence *)(untyped_member);
  return &member->data[index];
}

void dual_dynamixel_node__msg__SetPositions__rosidl_typesupport_introspection_c__fetch_function__SetPositions__id(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const uint8_t * item =
    ((const uint8_t *)
    dual_dynamixel_node__msg__SetPositions__rosidl_typesupport_introspection_c__get_const_function__SetPositions__id(untyped_member, index));
  uint8_t * value =
    (uint8_t *)(untyped_value);
  *value = *item;
}

void dual_dynamixel_node__msg__SetPositions__rosidl_typesupport_introspection_c__assign_function__SetPositions__id(
  void * untyped_member, size_t index, const void * untyped_value)
{
  uint8_t * item =
    ((uint8_t *)
    dual_dynamixel_node__msg__SetPositions__rosidl_typesupport_introspection_c__get_function__SetPositions__id(untyped_member, index));
  const uint8_t * value =
    (const uint8_t *)(untyped_value);
  *item = *value;
}

bool dual_dynamixel_node__msg__SetPositions__rosidl_typesupport_introspection_c__resize_function__SetPositions__id(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__uint8__Sequence * member =
    (rosidl_runtime_c__uint8__Sequence *)(untyped_member);
  rosidl_runtime_c__uint8__Sequence__fini(member);
  return rosidl_runtime_c__uint8__Sequence__init(member, size);
}

size_t dual_dynamixel_node__msg__SetPositions__rosidl_typesupport_introspection_c__size_function__SetPositions__position(
  const void * untyped_member)
{
  const rosidl_runtime_c__int32__Sequence * member =
    (const rosidl_runtime_c__int32__Sequence *)(untyped_member);
  return member->size;
}

const void * dual_dynamixel_node__msg__SetPositions__rosidl_typesupport_introspection_c__get_const_function__SetPositions__position(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__int32__Sequence * member =
    (const rosidl_runtime_c__int32__Sequence *)(untyped_member);
  return &member->data[index];
}

void * dual_dynamixel_node__msg__SetPositions__rosidl_typesupport_introspection_c__get_function__SetPositions__position(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__int32__Sequence * member =
    (rosidl_runtime_c__int32__Sequence *)(untyped_member);
  return &member->data[index];
}

void dual_dynamixel_node__msg__SetPositions__rosidl_typesupport_introspection_c__fetch_function__SetPositions__position(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const int32_t * item =
    ((const int32_t *)
    dual_dynamixel_node__msg__SetPositions__rosidl_typesupport_introspection_c__get_const_function__SetPositions__position(untyped_member, index));
  int32_t * value =
    (int32_t *)(untyped_value);
  *value = *item;
}

void dual_dynamixel_node__msg__SetPositions__rosidl_typesupport_introspection_c__assign_function__SetPositions__position(
  void * untyped_member, size_t index, const void * untyped_value)
{
  int32_t * item =
    ((int32_t *)
    dual_dynamixel_node__msg__SetPositions__rosidl_typesupport_introspection_c__get_function__SetPositions__position(untyped_member, index));
  const int32_t * value =
    (const int32_t *)(untyped_value);
  *item = *value;
}

bool dual_dynamixel_node__msg__SetPositions__rosidl_typesupport_introspection_c__resize_function__SetPositions__position(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__int32__Sequence * member =
    (rosidl_runtime_c__int32__Sequence *)(untyped_member);
  rosidl_runtime_c__int32__Sequence__fini(member);
  return rosidl_runtime_c__int32__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember dual_dynamixel_node__msg__SetPositions__rosidl_typesupport_introspection_c__SetPositions_message_member_array[2] = {
  {
    "id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dual_dynamixel_node__msg__SetPositions, id),  // bytes offset in struct
    NULL,  // default value
    dual_dynamixel_node__msg__SetPositions__rosidl_typesupport_introspection_c__size_function__SetPositions__id,  // size() function pointer
    dual_dynamixel_node__msg__SetPositions__rosidl_typesupport_introspection_c__get_const_function__SetPositions__id,  // get_const(index) function pointer
    dual_dynamixel_node__msg__SetPositions__rosidl_typesupport_introspection_c__get_function__SetPositions__id,  // get(index) function pointer
    dual_dynamixel_node__msg__SetPositions__rosidl_typesupport_introspection_c__fetch_function__SetPositions__id,  // fetch(index, &value) function pointer
    dual_dynamixel_node__msg__SetPositions__rosidl_typesupport_introspection_c__assign_function__SetPositions__id,  // assign(index, value) function pointer
    dual_dynamixel_node__msg__SetPositions__rosidl_typesupport_introspection_c__resize_function__SetPositions__id  // resize(index) function pointer
  },
  {
    "position",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dual_dynamixel_node__msg__SetPositions, position),  // bytes offset in struct
    NULL,  // default value
    dual_dynamixel_node__msg__SetPositions__rosidl_typesupport_introspection_c__size_function__SetPositions__position,  // size() function pointer
    dual_dynamixel_node__msg__SetPositions__rosidl_typesupport_introspection_c__get_const_function__SetPositions__position,  // get_const(index) function pointer
    dual_dynamixel_node__msg__SetPositions__rosidl_typesupport_introspection_c__get_function__SetPositions__position,  // get(index) function pointer
    dual_dynamixel_node__msg__SetPositions__rosidl_typesupport_introspection_c__fetch_function__SetPositions__position,  // fetch(index, &value) function pointer
    dual_dynamixel_node__msg__SetPositions__rosidl_typesupport_introspection_c__assign_function__SetPositions__position,  // assign(index, value) function pointer
    dual_dynamixel_node__msg__SetPositions__rosidl_typesupport_introspection_c__resize_function__SetPositions__position  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers dual_dynamixel_node__msg__SetPositions__rosidl_typesupport_introspection_c__SetPositions_message_members = {
  "dual_dynamixel_node__msg",  // message namespace
  "SetPositions",  // message name
  2,  // number of fields
  sizeof(dual_dynamixel_node__msg__SetPositions),
  dual_dynamixel_node__msg__SetPositions__rosidl_typesupport_introspection_c__SetPositions_message_member_array,  // message members
  dual_dynamixel_node__msg__SetPositions__rosidl_typesupport_introspection_c__SetPositions_init_function,  // function to initialize message memory (memory has to be allocated)
  dual_dynamixel_node__msg__SetPositions__rosidl_typesupport_introspection_c__SetPositions_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t dual_dynamixel_node__msg__SetPositions__rosidl_typesupport_introspection_c__SetPositions_message_type_support_handle = {
  0,
  &dual_dynamixel_node__msg__SetPositions__rosidl_typesupport_introspection_c__SetPositions_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_dual_dynamixel_node
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, dual_dynamixel_node, msg, SetPositions)() {
  if (!dual_dynamixel_node__msg__SetPositions__rosidl_typesupport_introspection_c__SetPositions_message_type_support_handle.typesupport_identifier) {
    dual_dynamixel_node__msg__SetPositions__rosidl_typesupport_introspection_c__SetPositions_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &dual_dynamixel_node__msg__SetPositions__rosidl_typesupport_introspection_c__SetPositions_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
