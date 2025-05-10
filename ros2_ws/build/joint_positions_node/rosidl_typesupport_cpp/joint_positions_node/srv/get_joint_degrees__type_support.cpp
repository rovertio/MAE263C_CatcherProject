// generated from rosidl_typesupport_cpp/resource/idl__type_support.cpp.em
// with input from joint_positions_node:srv/GetJointDegrees.idl
// generated code does not contain a copyright notice

#include "cstddef"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "joint_positions_node/srv/detail/get_joint_degrees__struct.hpp"
#include "rosidl_typesupport_cpp/identifier.hpp"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
#include "rosidl_typesupport_cpp/visibility_control.h"
#include "rosidl_typesupport_interface/macros.h"

namespace joint_positions_node
{

namespace srv
{

namespace rosidl_typesupport_cpp
{

typedef struct _GetJointDegrees_Request_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _GetJointDegrees_Request_type_support_ids_t;

static const _GetJointDegrees_Request_type_support_ids_t _GetJointDegrees_Request_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _GetJointDegrees_Request_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _GetJointDegrees_Request_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _GetJointDegrees_Request_type_support_symbol_names_t _GetJointDegrees_Request_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, joint_positions_node, srv, GetJointDegrees_Request)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, joint_positions_node, srv, GetJointDegrees_Request)),
  }
};

typedef struct _GetJointDegrees_Request_type_support_data_t
{
  void * data[2];
} _GetJointDegrees_Request_type_support_data_t;

static _GetJointDegrees_Request_type_support_data_t _GetJointDegrees_Request_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _GetJointDegrees_Request_message_typesupport_map = {
  2,
  "joint_positions_node",
  &_GetJointDegrees_Request_message_typesupport_ids.typesupport_identifier[0],
  &_GetJointDegrees_Request_message_typesupport_symbol_names.symbol_name[0],
  &_GetJointDegrees_Request_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t GetJointDegrees_Request_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_GetJointDegrees_Request_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace joint_positions_node

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<joint_positions_node::srv::GetJointDegrees_Request>()
{
  return &::joint_positions_node::srv::rosidl_typesupport_cpp::GetJointDegrees_Request_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, joint_positions_node, srv, GetJointDegrees_Request)() {
  return get_message_type_support_handle<joint_positions_node::srv::GetJointDegrees_Request>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "joint_positions_node/srv/detail/get_joint_degrees__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace joint_positions_node
{

namespace srv
{

namespace rosidl_typesupport_cpp
{

typedef struct _GetJointDegrees_Response_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _GetJointDegrees_Response_type_support_ids_t;

static const _GetJointDegrees_Response_type_support_ids_t _GetJointDegrees_Response_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _GetJointDegrees_Response_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _GetJointDegrees_Response_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _GetJointDegrees_Response_type_support_symbol_names_t _GetJointDegrees_Response_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, joint_positions_node, srv, GetJointDegrees_Response)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, joint_positions_node, srv, GetJointDegrees_Response)),
  }
};

typedef struct _GetJointDegrees_Response_type_support_data_t
{
  void * data[2];
} _GetJointDegrees_Response_type_support_data_t;

static _GetJointDegrees_Response_type_support_data_t _GetJointDegrees_Response_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _GetJointDegrees_Response_message_typesupport_map = {
  2,
  "joint_positions_node",
  &_GetJointDegrees_Response_message_typesupport_ids.typesupport_identifier[0],
  &_GetJointDegrees_Response_message_typesupport_symbol_names.symbol_name[0],
  &_GetJointDegrees_Response_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t GetJointDegrees_Response_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_GetJointDegrees_Response_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace joint_positions_node

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<joint_positions_node::srv::GetJointDegrees_Response>()
{
  return &::joint_positions_node::srv::rosidl_typesupport_cpp::GetJointDegrees_Response_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, joint_positions_node, srv, GetJointDegrees_Response)() {
  return get_message_type_support_handle<joint_positions_node::srv::GetJointDegrees_Response>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "joint_positions_node/srv/detail/get_joint_degrees__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_cpp/service_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace joint_positions_node
{

namespace srv
{

namespace rosidl_typesupport_cpp
{

typedef struct _GetJointDegrees_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _GetJointDegrees_type_support_ids_t;

static const _GetJointDegrees_type_support_ids_t _GetJointDegrees_service_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _GetJointDegrees_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _GetJointDegrees_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _GetJointDegrees_type_support_symbol_names_t _GetJointDegrees_service_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, joint_positions_node, srv, GetJointDegrees)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, joint_positions_node, srv, GetJointDegrees)),
  }
};

typedef struct _GetJointDegrees_type_support_data_t
{
  void * data[2];
} _GetJointDegrees_type_support_data_t;

static _GetJointDegrees_type_support_data_t _GetJointDegrees_service_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _GetJointDegrees_service_typesupport_map = {
  2,
  "joint_positions_node",
  &_GetJointDegrees_service_typesupport_ids.typesupport_identifier[0],
  &_GetJointDegrees_service_typesupport_symbol_names.symbol_name[0],
  &_GetJointDegrees_service_typesupport_data.data[0],
};

static const rosidl_service_type_support_t GetJointDegrees_service_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_GetJointDegrees_service_typesupport_map),
  ::rosidl_typesupport_cpp::get_service_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace joint_positions_node

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_service_type_support_t *
get_service_type_support_handle<joint_positions_node::srv::GetJointDegrees>()
{
  return &::joint_positions_node::srv::rosidl_typesupport_cpp::GetJointDegrees_service_type_support_handle;
}

}  // namespace rosidl_typesupport_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_cpp, joint_positions_node, srv, GetJointDegrees)() {
  return ::rosidl_typesupport_cpp::get_service_type_support_handle<joint_positions_node::srv::GetJointDegrees>();
}

#ifdef __cplusplus
}
#endif
