// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__rosidl_typesupport_fastrtps_cpp.hpp.em
// with input from dual_dynamixel_node:msg/SetPositions.idl
// generated code does not contain a copyright notice

#ifndef DUAL_DYNAMIXEL_NODE__MSG__DETAIL__SET_POSITIONS__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
#define DUAL_DYNAMIXEL_NODE__MSG__DETAIL__SET_POSITIONS__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "dual_dynamixel_node/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
#include "dual_dynamixel_node/msg/detail/set_positions__struct.hpp"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

#include "fastcdr/Cdr.h"

namespace dual_dynamixel_node
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_dual_dynamixel_node
cdr_serialize(
  const dual_dynamixel_node::msg::SetPositions & ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_dual_dynamixel_node
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  dual_dynamixel_node::msg::SetPositions & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_dual_dynamixel_node
get_serialized_size(
  const dual_dynamixel_node::msg::SetPositions & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_dual_dynamixel_node
max_serialized_size_SetPositions(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace dual_dynamixel_node

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_dual_dynamixel_node
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, dual_dynamixel_node, msg, SetPositions)();

#ifdef __cplusplus
}
#endif

#endif  // DUAL_DYNAMIXEL_NODE__MSG__DETAIL__SET_POSITIONS__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
