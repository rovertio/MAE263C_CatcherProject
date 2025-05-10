// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from joint_positions_node:msg/SetJointDegrees.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "joint_positions_node/msg/detail/set_joint_degrees__rosidl_typesupport_introspection_c.h"
#include "joint_positions_node/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "joint_positions_node/msg/detail/set_joint_degrees__functions.h"
#include "joint_positions_node/msg/detail/set_joint_degrees__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void joint_positions_node__msg__SetJointDegrees__rosidl_typesupport_introspection_c__SetJointDegrees_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  joint_positions_node__msg__SetJointDegrees__init(message_memory);
}

void joint_positions_node__msg__SetJointDegrees__rosidl_typesupport_introspection_c__SetJointDegrees_fini_function(void * message_memory)
{
  joint_positions_node__msg__SetJointDegrees__fini(message_memory);
}

size_t joint_positions_node__msg__SetJointDegrees__rosidl_typesupport_introspection_c__size_function__SetJointDegrees__degrees(
  const void * untyped_member)
{
  (void)untyped_member;
  return 2;
}

const void * joint_positions_node__msg__SetJointDegrees__rosidl_typesupport_introspection_c__get_const_function__SetJointDegrees__degrees(
  const void * untyped_member, size_t index)
{
  const double * member =
    (const double *)(untyped_member);
  return &member[index];
}

void * joint_positions_node__msg__SetJointDegrees__rosidl_typesupport_introspection_c__get_function__SetJointDegrees__degrees(
  void * untyped_member, size_t index)
{
  double * member =
    (double *)(untyped_member);
  return &member[index];
}

void joint_positions_node__msg__SetJointDegrees__rosidl_typesupport_introspection_c__fetch_function__SetJointDegrees__degrees(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    joint_positions_node__msg__SetJointDegrees__rosidl_typesupport_introspection_c__get_const_function__SetJointDegrees__degrees(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void joint_positions_node__msg__SetJointDegrees__rosidl_typesupport_introspection_c__assign_function__SetJointDegrees__degrees(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    joint_positions_node__msg__SetJointDegrees__rosidl_typesupport_introspection_c__get_function__SetJointDegrees__degrees(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

static rosidl_typesupport_introspection_c__MessageMember joint_positions_node__msg__SetJointDegrees__rosidl_typesupport_introspection_c__SetJointDegrees_message_member_array[1] = {
  {
    "degrees",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    2,  // array size
    false,  // is upper bound
    offsetof(joint_positions_node__msg__SetJointDegrees, degrees),  // bytes offset in struct
    NULL,  // default value
    joint_positions_node__msg__SetJointDegrees__rosidl_typesupport_introspection_c__size_function__SetJointDegrees__degrees,  // size() function pointer
    joint_positions_node__msg__SetJointDegrees__rosidl_typesupport_introspection_c__get_const_function__SetJointDegrees__degrees,  // get_const(index) function pointer
    joint_positions_node__msg__SetJointDegrees__rosidl_typesupport_introspection_c__get_function__SetJointDegrees__degrees,  // get(index) function pointer
    joint_positions_node__msg__SetJointDegrees__rosidl_typesupport_introspection_c__fetch_function__SetJointDegrees__degrees,  // fetch(index, &value) function pointer
    joint_positions_node__msg__SetJointDegrees__rosidl_typesupport_introspection_c__assign_function__SetJointDegrees__degrees,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers joint_positions_node__msg__SetJointDegrees__rosidl_typesupport_introspection_c__SetJointDegrees_message_members = {
  "joint_positions_node__msg",  // message namespace
  "SetJointDegrees",  // message name
  1,  // number of fields
  sizeof(joint_positions_node__msg__SetJointDegrees),
  joint_positions_node__msg__SetJointDegrees__rosidl_typesupport_introspection_c__SetJointDegrees_message_member_array,  // message members
  joint_positions_node__msg__SetJointDegrees__rosidl_typesupport_introspection_c__SetJointDegrees_init_function,  // function to initialize message memory (memory has to be allocated)
  joint_positions_node__msg__SetJointDegrees__rosidl_typesupport_introspection_c__SetJointDegrees_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t joint_positions_node__msg__SetJointDegrees__rosidl_typesupport_introspection_c__SetJointDegrees_message_type_support_handle = {
  0,
  &joint_positions_node__msg__SetJointDegrees__rosidl_typesupport_introspection_c__SetJointDegrees_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_joint_positions_node
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, joint_positions_node, msg, SetJointDegrees)() {
  if (!joint_positions_node__msg__SetJointDegrees__rosidl_typesupport_introspection_c__SetJointDegrees_message_type_support_handle.typesupport_identifier) {
    joint_positions_node__msg__SetJointDegrees__rosidl_typesupport_introspection_c__SetJointDegrees_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &joint_positions_node__msg__SetJointDegrees__rosidl_typesupport_introspection_c__SetJointDegrees_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
