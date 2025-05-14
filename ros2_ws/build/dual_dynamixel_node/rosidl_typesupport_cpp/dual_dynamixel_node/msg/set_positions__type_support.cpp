// generated from rosidl_typesupport_cpp/resource/idl__type_support.cpp.em
// with input from dual_dynamixel_node:msg/SetPositions.idl
// generated code does not contain a copyright notice

#include "cstddef"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "dual_dynamixel_node/msg/detail/set_positions__struct.hpp"
#include "rosidl_typesupport_cpp/identifier.hpp"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
#include "rosidl_typesupport_cpp/visibility_control.h"
#include "rosidl_typesupport_interface/macros.h"

namespace dual_dynamixel_node
{

namespace msg
{

namespace rosidl_typesupport_cpp
{

typedef struct _SetPositions_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _SetPositions_type_support_ids_t;

static const _SetPositions_type_support_ids_t _SetPositions_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _SetPositions_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _SetPositions_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _SetPositions_type_support_symbol_names_t _SetPositions_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, dual_dynamixel_node, msg, SetPositions)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, dual_dynamixel_node, msg, SetPositions)),
  }
};

typedef struct _SetPositions_type_support_data_t
{
  void * data[2];
} _SetPositions_type_support_data_t;

static _SetPositions_type_support_data_t _SetPositions_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _SetPositions_message_typesupport_map = {
  2,
  "dual_dynamixel_node",
  &_SetPositions_message_typesupport_ids.typesupport_identifier[0],
  &_SetPositions_message_typesupport_symbol_names.symbol_name[0],
  &_SetPositions_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t SetPositions_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_SetPositions_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace msg

}  // namespace dual_dynamixel_node

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<dual_dynamixel_node::msg::SetPositions>()
{
  return &::dual_dynamixel_node::msg::rosidl_typesupport_cpp::SetPositions_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, dual_dynamixel_node, msg, SetPositions)() {
  return get_message_type_support_handle<dual_dynamixel_node::msg::SetPositions>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp
