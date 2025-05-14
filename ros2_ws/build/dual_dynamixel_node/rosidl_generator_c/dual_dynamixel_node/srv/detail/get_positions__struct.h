// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from dual_dynamixel_node:srv/GetPositions.idl
// generated code does not contain a copyright notice

#ifndef DUAL_DYNAMIXEL_NODE__SRV__DETAIL__GET_POSITIONS__STRUCT_H_
#define DUAL_DYNAMIXEL_NODE__SRV__DETAIL__GET_POSITIONS__STRUCT_H_

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
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in srv/GetPositions in the package dual_dynamixel_node.
typedef struct dual_dynamixel_node__srv__GetPositions_Request
{
  rosidl_runtime_c__uint8__Sequence id;
} dual_dynamixel_node__srv__GetPositions_Request;

// Struct for a sequence of dual_dynamixel_node__srv__GetPositions_Request.
typedef struct dual_dynamixel_node__srv__GetPositions_Request__Sequence
{
  dual_dynamixel_node__srv__GetPositions_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} dual_dynamixel_node__srv__GetPositions_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'position'
// already included above
// #include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in srv/GetPositions in the package dual_dynamixel_node.
typedef struct dual_dynamixel_node__srv__GetPositions_Response
{
  rosidl_runtime_c__int32__Sequence position;
} dual_dynamixel_node__srv__GetPositions_Response;

// Struct for a sequence of dual_dynamixel_node__srv__GetPositions_Response.
typedef struct dual_dynamixel_node__srv__GetPositions_Response__Sequence
{
  dual_dynamixel_node__srv__GetPositions_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} dual_dynamixel_node__srv__GetPositions_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // DUAL_DYNAMIXEL_NODE__SRV__DETAIL__GET_POSITIONS__STRUCT_H_
