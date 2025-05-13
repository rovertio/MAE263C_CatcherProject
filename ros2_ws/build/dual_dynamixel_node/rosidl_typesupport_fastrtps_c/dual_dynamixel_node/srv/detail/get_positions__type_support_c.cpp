// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from dual_dynamixel_node:srv/GetPositions.idl
// generated code does not contain a copyright notice
#include "dual_dynamixel_node/srv/detail/get_positions__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "dual_dynamixel_node/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "dual_dynamixel_node/srv/detail/get_positions__struct.h"
#include "dual_dynamixel_node/srv/detail/get_positions__functions.h"
#include "fastcdr/Cdr.h"

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

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif

#include "rosidl_runtime_c/primitives_sequence.h"  // id
#include "rosidl_runtime_c/primitives_sequence_functions.h"  // id

// forward declare type support functions


using _GetPositions_Request__ros_msg_type = dual_dynamixel_node__srv__GetPositions_Request;

static bool _GetPositions_Request__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _GetPositions_Request__ros_msg_type * ros_message = static_cast<const _GetPositions_Request__ros_msg_type *>(untyped_ros_message);
  // Field name: id
  {
    size_t size = ros_message->id.size;
    auto array_ptr = ros_message->id.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serializeArray(array_ptr, size);
  }

  return true;
}

static bool _GetPositions_Request__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _GetPositions_Request__ros_msg_type * ros_message = static_cast<_GetPositions_Request__ros_msg_type *>(untyped_ros_message);
  // Field name: id
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    if (ros_message->id.data) {
      rosidl_runtime_c__uint8__Sequence__fini(&ros_message->id);
    }
    if (!rosidl_runtime_c__uint8__Sequence__init(&ros_message->id, size)) {
      fprintf(stderr, "failed to create array for field 'id'");
      return false;
    }
    auto array_ptr = ros_message->id.data;
    cdr.deserializeArray(array_ptr, size);
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_dual_dynamixel_node
size_t get_serialized_size_dual_dynamixel_node__srv__GetPositions_Request(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _GetPositions_Request__ros_msg_type * ros_message = static_cast<const _GetPositions_Request__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name id
  {
    size_t array_size = ros_message->id.size;
    auto array_ptr = ros_message->id.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _GetPositions_Request__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_dual_dynamixel_node__srv__GetPositions_Request(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_dual_dynamixel_node
size_t max_serialized_size_dual_dynamixel_node__srv__GetPositions_Request(
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

  // member: id
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = dual_dynamixel_node__srv__GetPositions_Request;
    is_plain =
      (
      offsetof(DataType, id) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _GetPositions_Request__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_dual_dynamixel_node__srv__GetPositions_Request(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_GetPositions_Request = {
  "dual_dynamixel_node::srv",
  "GetPositions_Request",
  _GetPositions_Request__cdr_serialize,
  _GetPositions_Request__cdr_deserialize,
  _GetPositions_Request__get_serialized_size,
  _GetPositions_Request__max_serialized_size
};

static rosidl_message_type_support_t _GetPositions_Request__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_GetPositions_Request,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, dual_dynamixel_node, srv, GetPositions_Request)() {
  return &_GetPositions_Request__type_support;
}

#if defined(__cplusplus)
}
#endif

// already included above
// #include <cassert>
// already included above
// #include <limits>
// already included above
// #include <string>
// already included above
// #include "rosidl_typesupport_fastrtps_c/identifier.h"
// already included above
// #include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
// already included above
// #include "dual_dynamixel_node/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
// already included above
// #include "dual_dynamixel_node/srv/detail/get_positions__struct.h"
// already included above
// #include "dual_dynamixel_node/srv/detail/get_positions__functions.h"
// already included above
// #include "fastcdr/Cdr.h"

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

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif

// already included above
// #include "rosidl_runtime_c/primitives_sequence.h"  // position
// already included above
// #include "rosidl_runtime_c/primitives_sequence_functions.h"  // position

// forward declare type support functions


using _GetPositions_Response__ros_msg_type = dual_dynamixel_node__srv__GetPositions_Response;

static bool _GetPositions_Response__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _GetPositions_Response__ros_msg_type * ros_message = static_cast<const _GetPositions_Response__ros_msg_type *>(untyped_ros_message);
  // Field name: position
  {
    size_t size = ros_message->position.size;
    auto array_ptr = ros_message->position.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serializeArray(array_ptr, size);
  }

  return true;
}

static bool _GetPositions_Response__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _GetPositions_Response__ros_msg_type * ros_message = static_cast<_GetPositions_Response__ros_msg_type *>(untyped_ros_message);
  // Field name: position
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    if (ros_message->position.data) {
      rosidl_runtime_c__int32__Sequence__fini(&ros_message->position);
    }
    if (!rosidl_runtime_c__int32__Sequence__init(&ros_message->position, size)) {
      fprintf(stderr, "failed to create array for field 'position'");
      return false;
    }
    auto array_ptr = ros_message->position.data;
    cdr.deserializeArray(array_ptr, size);
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_dual_dynamixel_node
size_t get_serialized_size_dual_dynamixel_node__srv__GetPositions_Response(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _GetPositions_Response__ros_msg_type * ros_message = static_cast<const _GetPositions_Response__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name position
  {
    size_t array_size = ros_message->position.size;
    auto array_ptr = ros_message->position.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _GetPositions_Response__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_dual_dynamixel_node__srv__GetPositions_Response(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_dual_dynamixel_node
size_t max_serialized_size_dual_dynamixel_node__srv__GetPositions_Response(
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

  // member: position
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = dual_dynamixel_node__srv__GetPositions_Response;
    is_plain =
      (
      offsetof(DataType, position) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _GetPositions_Response__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_dual_dynamixel_node__srv__GetPositions_Response(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_GetPositions_Response = {
  "dual_dynamixel_node::srv",
  "GetPositions_Response",
  _GetPositions_Response__cdr_serialize,
  _GetPositions_Response__cdr_deserialize,
  _GetPositions_Response__get_serialized_size,
  _GetPositions_Response__max_serialized_size
};

static rosidl_message_type_support_t _GetPositions_Response__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_GetPositions_Response,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, dual_dynamixel_node, srv, GetPositions_Response)() {
  return &_GetPositions_Response__type_support;
}

#if defined(__cplusplus)
}
#endif

#include "rosidl_typesupport_fastrtps_cpp/service_type_support.h"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_c/identifier.h"
// already included above
// #include "dual_dynamixel_node/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "dual_dynamixel_node/srv/get_positions.h"

#if defined(__cplusplus)
extern "C"
{
#endif

static service_type_support_callbacks_t GetPositions__callbacks = {
  "dual_dynamixel_node::srv",
  "GetPositions",
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, dual_dynamixel_node, srv, GetPositions_Request)(),
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, dual_dynamixel_node, srv, GetPositions_Response)(),
};

static rosidl_service_type_support_t GetPositions__handle = {
  rosidl_typesupport_fastrtps_c__identifier,
  &GetPositions__callbacks,
  get_service_typesupport_handle_function,
};

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, dual_dynamixel_node, srv, GetPositions)() {
  return &GetPositions__handle;
}

#if defined(__cplusplus)
}
#endif
