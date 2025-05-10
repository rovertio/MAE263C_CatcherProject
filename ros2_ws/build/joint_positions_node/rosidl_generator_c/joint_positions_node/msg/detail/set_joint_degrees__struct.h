// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from joint_positions_node:msg/SetJointDegrees.idl
// generated code does not contain a copyright notice

#ifndef JOINT_POSITIONS_NODE__MSG__DETAIL__SET_JOINT_DEGREES__STRUCT_H_
#define JOINT_POSITIONS_NODE__MSG__DETAIL__SET_JOINT_DEGREES__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/SetJointDegrees in the package joint_positions_node.
/**
  * Desired joint angles in degrees
 */
typedef struct joint_positions_node__msg__SetJointDegrees
{
  double degrees[2];
} joint_positions_node__msg__SetJointDegrees;

// Struct for a sequence of joint_positions_node__msg__SetJointDegrees.
typedef struct joint_positions_node__msg__SetJointDegrees__Sequence
{
  joint_positions_node__msg__SetJointDegrees * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} joint_positions_node__msg__SetJointDegrees__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // JOINT_POSITIONS_NODE__MSG__DETAIL__SET_JOINT_DEGREES__STRUCT_H_
