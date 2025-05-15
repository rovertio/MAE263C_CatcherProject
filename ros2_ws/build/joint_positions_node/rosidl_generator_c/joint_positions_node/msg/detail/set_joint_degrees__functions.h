// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from joint_positions_node:msg/SetJointDegrees.idl
// generated code does not contain a copyright notice

#ifndef JOINT_POSITIONS_NODE__MSG__DETAIL__SET_JOINT_DEGREES__FUNCTIONS_H_
#define JOINT_POSITIONS_NODE__MSG__DETAIL__SET_JOINT_DEGREES__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "joint_positions_node/msg/rosidl_generator_c__visibility_control.h"

#include "joint_positions_node/msg/detail/set_joint_degrees__struct.h"

/// Initialize msg/SetJointDegrees message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * joint_positions_node__msg__SetJointDegrees
 * )) before or use
 * joint_positions_node__msg__SetJointDegrees__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_joint_positions_node
bool
joint_positions_node__msg__SetJointDegrees__init(joint_positions_node__msg__SetJointDegrees * msg);

/// Finalize msg/SetJointDegrees message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_joint_positions_node
void
joint_positions_node__msg__SetJointDegrees__fini(joint_positions_node__msg__SetJointDegrees * msg);

/// Create msg/SetJointDegrees message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * joint_positions_node__msg__SetJointDegrees__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_joint_positions_node
joint_positions_node__msg__SetJointDegrees *
joint_positions_node__msg__SetJointDegrees__create();

/// Destroy msg/SetJointDegrees message.
/**
 * It calls
 * joint_positions_node__msg__SetJointDegrees__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_joint_positions_node
void
joint_positions_node__msg__SetJointDegrees__destroy(joint_positions_node__msg__SetJointDegrees * msg);

/// Check for msg/SetJointDegrees message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_joint_positions_node
bool
joint_positions_node__msg__SetJointDegrees__are_equal(const joint_positions_node__msg__SetJointDegrees * lhs, const joint_positions_node__msg__SetJointDegrees * rhs);

/// Copy a msg/SetJointDegrees message.
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
ROSIDL_GENERATOR_C_PUBLIC_joint_positions_node
bool
joint_positions_node__msg__SetJointDegrees__copy(
  const joint_positions_node__msg__SetJointDegrees * input,
  joint_positions_node__msg__SetJointDegrees * output);

/// Initialize array of msg/SetJointDegrees messages.
/**
 * It allocates the memory for the number of elements and calls
 * joint_positions_node__msg__SetJointDegrees__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_joint_positions_node
bool
joint_positions_node__msg__SetJointDegrees__Sequence__init(joint_positions_node__msg__SetJointDegrees__Sequence * array, size_t size);

/// Finalize array of msg/SetJointDegrees messages.
/**
 * It calls
 * joint_positions_node__msg__SetJointDegrees__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_joint_positions_node
void
joint_positions_node__msg__SetJointDegrees__Sequence__fini(joint_positions_node__msg__SetJointDegrees__Sequence * array);

/// Create array of msg/SetJointDegrees messages.
/**
 * It allocates the memory for the array and calls
 * joint_positions_node__msg__SetJointDegrees__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_joint_positions_node
joint_positions_node__msg__SetJointDegrees__Sequence *
joint_positions_node__msg__SetJointDegrees__Sequence__create(size_t size);

/// Destroy array of msg/SetJointDegrees messages.
/**
 * It calls
 * joint_positions_node__msg__SetJointDegrees__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_joint_positions_node
void
joint_positions_node__msg__SetJointDegrees__Sequence__destroy(joint_positions_node__msg__SetJointDegrees__Sequence * array);

/// Check for msg/SetJointDegrees message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_joint_positions_node
bool
joint_positions_node__msg__SetJointDegrees__Sequence__are_equal(const joint_positions_node__msg__SetJointDegrees__Sequence * lhs, const joint_positions_node__msg__SetJointDegrees__Sequence * rhs);

/// Copy an array of msg/SetJointDegrees messages.
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
ROSIDL_GENERATOR_C_PUBLIC_joint_positions_node
bool
joint_positions_node__msg__SetJointDegrees__Sequence__copy(
  const joint_positions_node__msg__SetJointDegrees__Sequence * input,
  joint_positions_node__msg__SetJointDegrees__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // JOINT_POSITIONS_NODE__MSG__DETAIL__SET_JOINT_DEGREES__FUNCTIONS_H_
