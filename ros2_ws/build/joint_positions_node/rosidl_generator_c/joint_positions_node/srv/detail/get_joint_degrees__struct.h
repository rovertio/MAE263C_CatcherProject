// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from joint_positions_node:srv/GetJointDegrees.idl
// generated code does not contain a copyright notice

#ifndef JOINT_POSITIONS_NODE__SRV__DETAIL__GET_JOINT_DEGREES__STRUCT_H_
#define JOINT_POSITIONS_NODE__SRV__DETAIL__GET_JOINT_DEGREES__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/GetJointDegrees in the package joint_positions_node.
typedef struct joint_positions_node__srv__GetJointDegrees_Request
{
  /// unused; just to satisfy ROS 2 service syntax
  bool dummy;
} joint_positions_node__srv__GetJointDegrees_Request;

// Struct for a sequence of joint_positions_node__srv__GetJointDegrees_Request.
typedef struct joint_positions_node__srv__GetJointDegrees_Request__Sequence
{
  joint_positions_node__srv__GetJointDegrees_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} joint_positions_node__srv__GetJointDegrees_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/GetJointDegrees in the package joint_positions_node.
typedef struct joint_positions_node__srv__GetJointDegrees_Response
{
  double degrees[2];
} joint_positions_node__srv__GetJointDegrees_Response;

// Struct for a sequence of joint_positions_node__srv__GetJointDegrees_Response.
typedef struct joint_positions_node__srv__GetJointDegrees_Response__Sequence
{
  joint_positions_node__srv__GetJointDegrees_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} joint_positions_node__srv__GetJointDegrees_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // JOINT_POSITIONS_NODE__SRV__DETAIL__GET_JOINT_DEGREES__STRUCT_H_
