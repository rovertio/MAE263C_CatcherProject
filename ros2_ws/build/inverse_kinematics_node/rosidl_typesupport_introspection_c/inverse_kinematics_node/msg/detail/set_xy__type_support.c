// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from inverse_kinematics_node:msg/SetXY.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "inverse_kinematics_node/msg/detail/set_xy__rosidl_typesupport_introspection_c.h"
#include "inverse_kinematics_node/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "inverse_kinematics_node/msg/detail/set_xy__functions.h"
#include "inverse_kinematics_node/msg/detail/set_xy__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void inverse_kinematics_node__msg__SetXY__rosidl_typesupport_introspection_c__SetXY_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  inverse_kinematics_node__msg__SetXY__init(message_memory);
}

void inverse_kinematics_node__msg__SetXY__rosidl_typesupport_introspection_c__SetXY_fini_function(void * message_memory)
{
  inverse_kinematics_node__msg__SetXY__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember inverse_kinematics_node__msg__SetXY__rosidl_typesupport_introspection_c__SetXY_message_member_array[2] = {
  {
    "x",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(inverse_kinematics_node__msg__SetXY, x),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "y",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(inverse_kinematics_node__msg__SetXY, y),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers inverse_kinematics_node__msg__SetXY__rosidl_typesupport_introspection_c__SetXY_message_members = {
  "inverse_kinematics_node__msg",  // message namespace
  "SetXY",  // message name
  2,  // number of fields
  sizeof(inverse_kinematics_node__msg__SetXY),
  inverse_kinematics_node__msg__SetXY__rosidl_typesupport_introspection_c__SetXY_message_member_array,  // message members
  inverse_kinematics_node__msg__SetXY__rosidl_typesupport_introspection_c__SetXY_init_function,  // function to initialize message memory (memory has to be allocated)
  inverse_kinematics_node__msg__SetXY__rosidl_typesupport_introspection_c__SetXY_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t inverse_kinematics_node__msg__SetXY__rosidl_typesupport_introspection_c__SetXY_message_type_support_handle = {
  0,
  &inverse_kinematics_node__msg__SetXY__rosidl_typesupport_introspection_c__SetXY_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_inverse_kinematics_node
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, inverse_kinematics_node, msg, SetXY)() {
  if (!inverse_kinematics_node__msg__SetXY__rosidl_typesupport_introspection_c__SetXY_message_type_support_handle.typesupport_identifier) {
    inverse_kinematics_node__msg__SetXY__rosidl_typesupport_introspection_c__SetXY_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &inverse_kinematics_node__msg__SetXY__rosidl_typesupport_introspection_c__SetXY_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
