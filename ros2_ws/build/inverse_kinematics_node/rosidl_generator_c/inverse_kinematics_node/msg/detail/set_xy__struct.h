// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from inverse_kinematics_node:msg/SetXY.idl
// generated code does not contain a copyright notice

#ifndef INVERSE_KINEMATICS_NODE__MSG__DETAIL__SET_XY__STRUCT_H_
#define INVERSE_KINEMATICS_NODE__MSG__DETAIL__SET_XY__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/SetXY in the package inverse_kinematics_node.
typedef struct inverse_kinematics_node__msg__SetXY
{
  double x;
  double y;
} inverse_kinematics_node__msg__SetXY;

// Struct for a sequence of inverse_kinematics_node__msg__SetXY.
typedef struct inverse_kinematics_node__msg__SetXY__Sequence
{
  inverse_kinematics_node__msg__SetXY * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} inverse_kinematics_node__msg__SetXY__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // INVERSE_KINEMATICS_NODE__MSG__DETAIL__SET_XY__STRUCT_H_
