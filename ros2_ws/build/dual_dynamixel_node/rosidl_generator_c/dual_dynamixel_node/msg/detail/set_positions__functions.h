// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from dual_dynamixel_node:msg/SetPositions.idl
// generated code does not contain a copyright notice

#ifndef DUAL_DYNAMIXEL_NODE__MSG__DETAIL__SET_POSITIONS__FUNCTIONS_H_
#define DUAL_DYNAMIXEL_NODE__MSG__DETAIL__SET_POSITIONS__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "dual_dynamixel_node/msg/rosidl_generator_c__visibility_control.h"

#include "dual_dynamixel_node/msg/detail/set_positions__struct.h"

/// Initialize msg/SetPositions message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * dual_dynamixel_node__msg__SetPositions
 * )) before or use
 * dual_dynamixel_node__msg__SetPositions__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_dual_dynamixel_node
bool
dual_dynamixel_node__msg__SetPositions__init(dual_dynamixel_node__msg__SetPositions * msg);

/// Finalize msg/SetPositions message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_dual_dynamixel_node
void
dual_dynamixel_node__msg__SetPositions__fini(dual_dynamixel_node__msg__SetPositions * msg);

/// Create msg/SetPositions message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * dual_dynamixel_node__msg__SetPositions__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_dual_dynamixel_node
dual_dynamixel_node__msg__SetPositions *
dual_dynamixel_node__msg__SetPositions__create();

/// Destroy msg/SetPositions message.
/**
 * It calls
 * dual_dynamixel_node__msg__SetPositions__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_dual_dynamixel_node
void
dual_dynamixel_node__msg__SetPositions__destroy(dual_dynamixel_node__msg__SetPositions * msg);

/// Check for msg/SetPositions message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_dual_dynamixel_node
bool
dual_dynamixel_node__msg__SetPositions__are_equal(const dual_dynamixel_node__msg__SetPositions * lhs, const dual_dynamixel_node__msg__SetPositions * rhs);

/// Copy a msg/SetPositions message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_dual_dynamixel_node
bool
dual_dynamixel_node__msg__SetPositions__copy(
  const dual_dynamixel_node__msg__SetPositions * input,
  dual_dynamixel_node__msg__SetPositions * output);

/// Initialize array of msg/SetPositions messages.
/**
 * It allocates the memory for the number of elements and calls
 * dual_dynamixel_node__msg__SetPositions__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_dual_dynamixel_node
bool
dual_dynamixel_node__msg__SetPositions__Sequence__init(dual_dynamixel_node__msg__SetPositions__Sequence * array, size_t size);

/// Finalize array of msg/SetPositions messages.
/**
 * It calls
 * dual_dynamixel_node__msg__SetPositions__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_dual_dynamixel_node
void
dual_dynamixel_node__msg__SetPositions__Sequence__fini(dual_dynamixel_node__msg__SetPositions__Sequence * array);

/// Create array of msg/SetPositions messages.
/**
 * It allocates the memory for the array and calls
 * dual_dynamixel_node__msg__SetPositions__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_dual_dynamixel_node
dual_dynamixel_node__msg__SetPositions__Sequence *
dual_dynamixel_node__msg__SetPositions__Sequence__create(size_t size);

/// Destroy array of msg/SetPositions messages.
/**
 * It calls
 * dual_dynamixel_node__msg__SetPositions__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_dual_dynamixel_node
void
dual_dynamixel_node__msg__SetPositions__Sequence__destroy(dual_dynamixel_node__msg__SetPositions__Sequence * array);

/// Check for msg/SetPositions message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_dual_dynamixel_node
bool
dual_dynamixel_node__msg__SetPositions__Sequence__are_equal(const dual_dynamixel_node__msg__SetPositions__Sequence * lhs, const dual_dynamixel_node__msg__SetPositions__Sequence * rhs);

/// Copy an array of msg/SetPositions messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_dual_dynamixel_node
bool
dual_dynamixel_node__msg__SetPositions__Sequence__copy(
  const dual_dynamixel_node__msg__SetPositions__Sequence * input,
  dual_dynamixel_node__msg__SetPositions__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // DUAL_DYNAMIXEL_NODE__MSG__DETAIL__SET_POSITIONS__FUNCTIONS_H_
