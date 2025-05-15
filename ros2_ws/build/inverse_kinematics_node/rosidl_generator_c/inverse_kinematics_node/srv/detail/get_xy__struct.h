// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from inverse_kinematics_node:srv/GetXY.idl
// generated code does not contain a copyright notice

#ifndef INVERSE_KINEMATICS_NODE__SRV__DETAIL__GET_XY__STRUCT_H_
#define INVERSE_KINEMATICS_NODE__SRV__DETAIL__GET_XY__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/GetXY in the package inverse_kinematics_node.
typedef struct inverse_kinematics_node__srv__GetXY_Request
{
  uint8_t structure_needs_at_least_one_member;
} inverse_kinematics_node__srv__GetXY_Request;

// Struct for a sequence of inverse_kinematics_node__srv__GetXY_Request.
typedef struct inverse_kinematics_node__srv__GetXY_Request__Sequence
{
  inverse_kinematics_node__srv__GetXY_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} inverse_kinematics_node__srv__GetXY_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/GetXY in the package inverse_kinematics_node.
typedef struct inverse_kinematics_node__srv__GetXY_Response
{
  double x;
  double y;
} inverse_kinematics_node__srv__GetXY_Response;

// Struct for a sequence of inverse_kinematics_node__srv__GetXY_Response.
typedef struct inverse_kinematics_node__srv__GetXY_Response__Sequence
{
  inverse_kinematics_node__srv__GetXY_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} inverse_kinematics_node__srv__GetXY_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // INVERSE_KINEMATICS_NODE__SRV__DETAIL__GET_XY__STRUCT_H_
