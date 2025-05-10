// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from joint_positions_node:srv/GetJointDegrees.idl
// generated code does not contain a copyright notice
#include "joint_positions_node/srv/detail/get_joint_degrees__rosidl_typesupport_fastrtps_cpp.hpp"
#include "joint_positions_node/srv/detail/get_joint_degrees__struct.hpp"

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

namespace srv
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_joint_positions_node
cdr_serialize(
  const joint_positions_node::srv::GetJointDegrees_Request & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: dummy
  cdr << (ros_message.dummy ? true : false);
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_joint_positions_node
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  joint_positions_node::srv::GetJointDegrees_Request & ros_message)
{
  // Member: dummy
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.dummy = tmp ? true : false;
  }

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_joint_positions_node
get_serialized_size(
  const joint_positions_node::srv::GetJointDegrees_Request & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: dummy
  {
    size_t item_size = sizeof(ros_message.dummy);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_joint_positions_node
max_serialized_size_GetJointDegrees_Request(
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


  // Member: dummy
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = joint_positions_node::srv::GetJointDegrees_Request;
    is_plain =
      (
      offsetof(DataType, dummy) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static bool _GetJointDegrees_Request__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const joint_positions_node::srv::GetJointDegrees_Request *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _GetJointDegrees_Request__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<joint_positions_node::srv::GetJointDegrees_Request *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _GetJointDegrees_Request__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const joint_positions_node::srv::GetJointDegrees_Request *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _GetJointDegrees_Request__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_GetJointDegrees_Request(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _GetJointDegrees_Request__callbacks = {
  "joint_positions_node::srv",
  "GetJointDegrees_Request",
  _GetJointDegrees_Request__cdr_serialize,
  _GetJointDegrees_Request__cdr_deserialize,
  _GetJointDegrees_Request__get_serialized_size,
  _GetJointDegrees_Request__max_serialized_size
};

static rosidl_message_type_support_t _GetJointDegrees_Request__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_GetJointDegrees_Request__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace srv

}  // namespace joint_positions_node

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_joint_positions_node
const rosidl_message_type_support_t *
get_message_type_support_handle<joint_positions_node::srv::GetJointDegrees_Request>()
{
  return &joint_positions_node::srv::typesupport_fastrtps_cpp::_GetJointDegrees_Request__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, joint_positions_node, srv, GetJointDegrees_Request)() {
  return &joint_positions_node::srv::typesupport_fastrtps_cpp::_GetJointDegrees_Request__handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include <limits>
// already included above
// #include <stdexcept>
// already included above
// #include <string>
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
// already included above
// #include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
// already included above
// #include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions

namespace joint_positions_node
{

namespace srv
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_joint_positions_node
cdr_serialize(
  const joint_positions_node::srv::GetJointDegrees_Response & ros_message,
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
  joint_positions_node::srv::GetJointDegrees_Response & ros_message)
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
  const joint_positions_node::srv::GetJointDegrees_Response & ros_message,
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
max_serialized_size_GetJointDegrees_Response(
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
    using DataType = joint_positions_node::srv::GetJointDegrees_Response;
    is_plain =
      (
      offsetof(DataType, degrees) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static bool _GetJointDegrees_Response__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const joint_positions_node::srv::GetJointDegrees_Response *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _GetJointDegrees_Response__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<joint_positions_node::srv::GetJointDegrees_Response *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _GetJointDegrees_Response__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const joint_positions_node::srv::GetJointDegrees_Response *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _GetJointDegrees_Response__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_GetJointDegrees_Response(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _GetJointDegrees_Response__callbacks = {
  "joint_positions_node::srv",
  "GetJointDegrees_Response",
  _GetJointDegrees_Response__cdr_serialize,
  _GetJointDegrees_Response__cdr_deserialize,
  _GetJointDegrees_Response__get_serialized_size,
  _GetJointDegrees_Response__max_serialized_size
};

static rosidl_message_type_support_t _GetJointDegrees_Response__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_GetJointDegrees_Response__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace srv

}  // namespace joint_positions_node

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_joint_positions_node
const rosidl_message_type_support_t *
get_message_type_support_handle<joint_positions_node::srv::GetJointDegrees_Response>()
{
  return &joint_positions_node::srv::typesupport_fastrtps_cpp::_GetJointDegrees_Response__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, joint_positions_node, srv, GetJointDegrees_Response)() {
  return &joint_positions_node::srv::typesupport_fastrtps_cpp::_GetJointDegrees_Response__handle;
}

#ifdef __cplusplus
}
#endif

#include "rmw/error_handling.h"
// already included above
// #include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/service_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/service_type_support_decl.hpp"

namespace joint_positions_node
{

namespace srv
{

namespace typesupport_fastrtps_cpp
{

static service_type_support_callbacks_t _GetJointDegrees__callbacks = {
  "joint_positions_node::srv",
  "GetJointDegrees",
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, joint_positions_node, srv, GetJointDegrees_Request)(),
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, joint_positions_node, srv, GetJointDegrees_Response)(),
};

static rosidl_service_type_support_t _GetJointDegrees__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_GetJointDegrees__callbacks,
  get_service_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace srv

}  // namespace joint_positions_node

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_joint_positions_node
const rosidl_service_type_support_t *
get_service_type_support_handle<joint_positions_node::srv::GetJointDegrees>()
{
  return &joint_positions_node::srv::typesupport_fastrtps_cpp::_GetJointDegrees__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, joint_positions_node, srv, GetJointDegrees)() {
  return &joint_positions_node::srv::typesupport_fastrtps_cpp::_GetJointDegrees__handle;
}

#ifdef __cplusplus
}
#endif
