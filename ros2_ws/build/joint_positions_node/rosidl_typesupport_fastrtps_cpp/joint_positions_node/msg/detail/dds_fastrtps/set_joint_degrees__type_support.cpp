// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from joint_positions_node:msg/SetJointDegrees.idl
// generated code does not contain a copyright notice
#include "joint_positions_node/msg/detail/set_joint_degrees__rosidl_typesupport_fastrtps_cpp.hpp"
#include "joint_positions_node/msg/detail/set_joint_degrees__struct.hpp"

#include <limits>
#include <stdexcept>
#include <string>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
#include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions

namespace joint_positions_node
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_joint_positions_node
cdr_serialize(
  const joint_positions_node::msg::SetJointDegrees & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: degrees
  {
    cdr << ros_message.degrees;
  }
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_joint_positions_node
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  joint_positions_node::msg::SetJointDegrees & ros_message)
{
  // Member: degrees
  {
    cdr >> ros_message.degrees;
  }

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_joint_positions_node
get_serialized_size(
  const joint_positions_node::msg::SetJointDegrees & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: degrees
  {
    size_t array_size = 2;
    size_t item_size = sizeof(ros_message.degrees[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_joint_positions_node
max_serialized_size_SetJointDegrees(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;


  // Member: degrees
  {
    size_t array_size = 2;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = joint_positions_node::msg::SetJointDegrees;
    is_plain =
      (
      offsetof(DataType, degrees) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static bool _SetJointDegrees__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const joint_positions_node::msg::SetJointDegrees *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _SetJointDegrees__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<joint_positions_node::msg::SetJointDegrees *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _SetJointDegrees__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const joint_positions_node::msg::SetJointDegrees *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _SetJointDegrees__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_SetJointDegrees(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _SetJointDegrees__callbacks = {
  "joint_positions_node::msg",
  "SetJointDegrees",
  _SetJointDegrees__cdr_serialize,
  _SetJointDegrees__cdr_deserialize,
  _SetJointDegrees__get_serialized_size,
  _SetJointDegrees__max_serialized_size
};

static rosidl_message_type_support_t _SetJointDegrees__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_SetJointDegrees__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace joint_positions_node

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_joint_positions_node
const rosidl_message_type_support_t *
get_message_type_support_handle<joint_positions_node::msg::SetJointDegrees>()
{
  return &joint_positions_node::msg::typesupport_fastrtps_cpp::_SetJointDegrees__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, joint_positions_node, msg, SetJointDegrees)() {
  return &joint_positions_node::msg::typesupport_fastrtps_cpp::_SetJointDegrees__handle;
}

#ifdef __cplusplus
}
#endif
