// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from inverse_kinematics_node:srv/GetXY.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "inverse_kinematics_node/srv/detail/get_xy__rosidl_typesupport_introspection_c.h"
#include "inverse_kinematics_node/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "inverse_kinematics_node/srv/detail/get_xy__functions.h"
#include "inverse_kinematics_node/srv/detail/get_xy__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void inverse_kinematics_node__srv__GetXY_Request__rosidl_typesupport_introspection_c__GetXY_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  inverse_kinematics_node__srv__GetXY_Request__init(message_memory);
}

void inverse_kinematics_node__srv__GetXY_Request__rosidl_typesupport_introspection_c__GetXY_Request_fini_function(void * message_memory)
{
  inverse_kinematics_node__srv__GetXY_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember inverse_kinematics_node__srv__GetXY_Request__rosidl_typesupport_introspection_c__GetXY_Request_message_member_array[1] = {
  {
    "structure_needs_at_least_one_member",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(inverse_kinematics_node__srv__GetXY_Request, structure_needs_at_least_one_member),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers inverse_kinematics_node__srv__GetXY_Request__rosidl_typesupport_introspection_c__GetXY_Request_message_members = {
  "inverse_kinematics_node__srv",  // message namespace
  "GetXY_Request",  // message name
  1,  // number of fields
  sizeof(inverse_kinematics_node__srv__GetXY_Request),
  inverse_kinematics_node__srv__GetXY_Request__rosidl_typesupport_introspection_c__GetXY_Request_message_member_array,  // message members
  inverse_kinematics_node__srv__GetXY_Request__rosidl_typesupport_introspection_c__GetXY_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  inverse_kinematics_node__srv__GetXY_Request__rosidl_typesupport_introspection_c__GetXY_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t inverse_kinematics_node__srv__GetXY_Request__rosidl_typesupport_introspection_c__GetXY_Request_message_type_support_handle = {
  0,
  &inverse_kinematics_node__srv__GetXY_Request__rosidl_typesupport_introspection_c__GetXY_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_inverse_kinematics_node
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, inverse_kinematics_node, srv, GetXY_Request)() {
  if (!inverse_kinematics_node__srv__GetXY_Request__rosidl_typesupport_introspection_c__GetXY_Request_message_type_support_handle.typesupport_identifier) {
    inverse_kinematics_node__srv__GetXY_Request__rosidl_typesupport_introspection_c__GetXY_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &inverse_kinematics_node__srv__GetXY_Request__rosidl_typesupport_introspection_c__GetXY_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "inverse_kinematics_node/srv/detail/get_xy__rosidl_typesupport_introspection_c.h"
// already included above
// #include "inverse_kinematics_node/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "inverse_kinematics_node/srv/detail/get_xy__functions.h"
// already included above
// #include "inverse_kinematics_node/srv/detail/get_xy__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void inverse_kinematics_node__srv__GetXY_Response__rosidl_typesupport_introspection_c__GetXY_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  inverse_kinematics_node__srv__GetXY_Response__init(message_memory);
}

void inverse_kinematics_node__srv__GetXY_Response__rosidl_typesupport_introspection_c__GetXY_Response_fini_function(void * message_memory)
{
  inverse_kinematics_node__srv__GetXY_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember inverse_kinematics_node__srv__GetXY_Response__rosidl_typesupport_introspection_c__GetXY_Response_message_member_array[2] = {
  {
    "x",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(inverse_kinematics_node__srv__GetXY_Response, x),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "y",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(inverse_kinematics_node__srv__GetXY_Response, y),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers inverse_kinematics_node__srv__GetXY_Response__rosidl_typesupport_introspection_c__GetXY_Response_message_members = {
  "inverse_kinematics_node__srv",  // message namespace
  "GetXY_Response",  // message name
  2,  // number of fields
  sizeof(inverse_kinematics_node__srv__GetXY_Response),
  inverse_kinematics_node__srv__GetXY_Response__rosidl_typesupport_introspection_c__GetXY_Response_message_member_array,  // message members
  inverse_kinematics_node__srv__GetXY_Response__rosidl_typesupport_introspection_c__GetXY_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  inverse_kinematics_node__srv__GetXY_Response__rosidl_typesupport_introspection_c__GetXY_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t inverse_kinematics_node__srv__GetXY_Response__rosidl_typesupport_introspection_c__GetXY_Response_message_type_support_handle = {
  0,
  &inverse_kinematics_node__srv__GetXY_Response__rosidl_typesupport_introspection_c__GetXY_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_inverse_kinematics_node
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, inverse_kinematics_node, srv, GetXY_Response)() {
  if (!inverse_kinematics_node__srv__GetXY_Response__rosidl_typesupport_introspection_c__GetXY_Response_message_type_support_handle.typesupport_identifier) {
    inverse_kinematics_node__srv__GetXY_Response__rosidl_typesupport_introspection_c__GetXY_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &inverse_kinematics_node__srv__GetXY_Response__rosidl_typesupport_introspection_c__GetXY_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "inverse_kinematics_node/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "inverse_kinematics_node/srv/detail/get_xy__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers inverse_kinematics_node__srv__detail__get_xy__rosidl_typesupport_introspection_c__GetXY_service_members = {
  "inverse_kinematics_node__srv",  // service namespace
  "GetXY",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // inverse_kinematics_node__srv__detail__get_xy__rosidl_typesupport_introspection_c__GetXY_Request_message_type_support_handle,
  NULL  // response message
  // inverse_kinematics_node__srv__detail__get_xy__rosidl_typesupport_introspection_c__GetXY_Response_message_type_support_handle
};

static rosidl_service_type_support_t inverse_kinematics_node__srv__detail__get_xy__rosidl_typesupport_introspection_c__GetXY_service_type_support_handle = {
  0,
  &inverse_kinematics_node__srv__detail__get_xy__rosidl_typesupport_introspection_c__GetXY_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, inverse_kinematics_node, srv, GetXY_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, inverse_kinematics_node, srv, GetXY_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_inverse_kinematics_node
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, inverse_kinematics_node, srv, GetXY)() {
  if (!inverse_kinematics_node__srv__detail__get_xy__rosidl_typesupport_introspection_c__GetXY_service_type_support_handle.typesupport_identifier) {
    inverse_kinematics_node__srv__detail__get_xy__rosidl_typesupport_introspection_c__GetXY_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)inverse_kinematics_node__srv__detail__get_xy__rosidl_typesupport_introspection_c__GetXY_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, inverse_kinematics_node, srv, GetXY_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, inverse_kinematics_node, srv, GetXY_Response)()->data;
  }

  return &inverse_kinematics_node__srv__detail__get_xy__rosidl_typesupport_introspection_c__GetXY_service_type_support_handle;
}
