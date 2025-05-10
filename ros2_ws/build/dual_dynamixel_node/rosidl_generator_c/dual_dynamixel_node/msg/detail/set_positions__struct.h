// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from dual_dynamixel_node:msg/SetPositions.idl
// generated code does not contain a copyright notice

#ifndef DUAL_DYNAMIXEL_NODE__MSG__DETAIL__SET_POSITIONS__STRUCT_H_
#define DUAL_DYNAMIXEL_NODE__MSG__DETAIL__SET_POSITIONS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'id'
// Member 'position'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in msg/SetPositions in the package dual_dynamixel_node.
/**
  * Command two motors at once
 */
typedef struct dual_dynamixel_node__msg__SetPositions
{
  rosidl_runtime_c__uint8__Sequence id;
  rosidl_runtime_c__int32__Sequence position;
} dual_dynamixel_node__msg__SetPositions;

// Struct for a sequence of dual_dynamixel_node__msg__SetPositions.
typedef struct dual_dynamixel_node__msg__SetPositions__Sequence
{
  dual_dynamixel_node__msg__SetPositions * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} dual_dynamixel_node__msg__SetPositions__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // DUAL_DYNAMIXEL_NODE__MSG__DETAIL__SET_POSITIONS__STRUCT_H_
