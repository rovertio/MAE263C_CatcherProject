// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from joint_positions_node:srv/GetJointDegrees.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "joint_positions_node/srv/detail/get_joint_degrees__rosidl_typesupport_introspection_c.h"
#include "joint_positions_node/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "joint_positions_node/srv/detail/get_joint_degrees__functions.h"
#include "joint_positions_node/srv/detail/get_joint_degrees__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void joint_positions_node__srv__GetJointDegrees_Request__rosidl_typesupport_introspection_c__GetJointDegrees_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  joint_positions_node__srv__GetJointDegrees_Request__init(message_memory);
}

void joint_positions_node__srv__GetJointDegrees_Request__rosidl_typesupport_introspection_c__GetJointDegrees_Request_fini_function(void * message_memory)
{
  joint_positions_node__srv__GetJointDegrees_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember joint_positions_node__srv__GetJointDegrees_Request__rosidl_typesupport_introspection_c__GetJointDegrees_Request_message_member_array[1] = {
  {
    "dummy",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(joint_positions_node__srv__GetJointDegrees_Request, dummy),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers joint_positions_node__srv__GetJointDegrees_Request__rosidl_typesupport_introspection_c__GetJointDegrees_Request_message_members = {
  "joint_positions_node__srv",  // message namespace
  "GetJointDegrees_Request",  // message name
  1,  // number of fields
  sizeof(joint_positions_node__srv__GetJointDegrees_Request),
  joint_positions_node__srv__GetJointDegrees_Request__rosidl_typesupport_introspection_c__GetJointDegrees_Request_message_member_array,  // message members
  joint_positions_node__srv__GetJointDegrees_Request__rosidl_typesupport_introspection_c__GetJointDegrees_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  joint_positions_node__srv__GetJointDegrees_Request__rosidl_typesupport_introspection_c__GetJointDegrees_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t joint_positions_node__srv__GetJointDegrees_Request__rosidl_typesupport_introspection_c__GetJointDegrees_Request_message_type_support_handle = {
  0,
  &joint_positions_node__srv__GetJointDegrees_Request__rosidl_typesupport_introspection_c__GetJointDegrees_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_joint_positions_node
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, joint_positions_node, srv, GetJointDegrees_Request)() {
  if (!joint_positions_node__srv__GetJointDegrees_Request__rosidl_typesupport_introspection_c__GetJointDegrees_Request_message_type_support_handle.typesupport_identifier) {
    joint_positions_node__srv__GetJointDegrees_Request__rosidl_typesupport_introspection_c__GetJointDegrees_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &joint_positions_node__srv__GetJointDegrees_Request__rosidl_typesupport_introspection_c__GetJointDegrees_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "joint_positions_node/srv/detail/get_joint_degrees__rosidl_typesupport_introspection_c.h"
// already included above
// #include "joint_positions_node/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "joint_positions_node/srv/detail/get_joint_degrees__functions.h"
// already included above
// #include "joint_positions_node/srv/detail/get_joint_degrees__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void joint_positions_node__srv__GetJointDegrees_Response__rosidl_typesupport_introspection_c__GetJointDegrees_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  joint_positions_node__srv__GetJointDegrees_Response__init(message_memory);
}

void joint_positions_node__srv__GetJointDegrees_Response__rosidl_typesupport_introspection_c__GetJointDegrees_Response_fini_function(void * message_memory)
{
  joint_positions_node__srv__GetJointDegrees_Response__fini(message_memory);
}

size_t joint_positions_node__srv__GetJointDegrees_Response__rosidl_typesupport_introspection_c__size_function__GetJointDegrees_Response__degrees(
  const void * untyped_member)
{
  (void)untyped_member;
  return 2;
}

const void * joint_positions_node__srv__GetJointDegrees_Response__rosidl_typesupport_introspection_c__get_const_function__GetJointDegrees_Response__degrees(
  const void * untyped_member, size_t index)
{
  const double * member =
    (const double *)(untyped_member);
  return &member[index];
}

void * joint_positions_node__srv__GetJointDegrees_Response__rosidl_typesupport_introspection_c__get_function__GetJointDegrees_Response__degrees(
  void * untyped_member, size_t index)
{
  double * member =
    (double *)(untyped_member);
  return &member[index];
}

void joint_positions_node__srv__GetJointDegrees_Response__rosidl_typesupport_introspection_c__fetch_function__GetJointDegrees_Response__degrees(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    joint_positions_node__srv__GetJointDegrees_Response__rosidl_typesupport_introspection_c__get_const_function__GetJointDegrees_Response__degrees(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void joint_positions_node__srv__GetJointDegrees_Response__rosidl_typesupport_introspection_c__assign_function__GetJointDegrees_Response__degrees(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    joint_positions_node__srv__GetJointDegrees_Response__rosidl_typesupport_introspection_c__get_function__GetJointDegrees_Response__degrees(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

static rosidl_typesupport_introspection_c__MessageMember joint_positions_node__srv__GetJointDegrees_Response__rosidl_typesupport_introspection_c__GetJointDegrees_Response_message_member_array[1] = {
  {
    "degrees",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    2,  // array size
    false,  // is upper bound
    offsetof(joint_positions_node__srv__GetJointDegrees_Response, degrees),  // bytes offset in struct
    NULL,  // default value
    joint_positions_node__srv__GetJointDegrees_Response__rosidl_typesupport_introspection_c__size_function__GetJointDegrees_Response__degrees,  // size() function pointer
    joint_positions_node__srv__GetJointDegrees_Response__rosidl_typesupport_introspection_c__get_const_function__GetJointDegrees_Response__degrees,  // get_const(index) function pointer
    joint_positions_node__srv__GetJointDegrees_Response__rosidl_typesupport_introspection_c__get_function__GetJointDegrees_Response__degrees,  // get(index) function pointer
    joint_positions_node__srv__GetJointDegrees_Response__rosidl_typesupport_introspection_c__fetch_function__GetJointDegrees_Response__degrees,  // fetch(index, &value) function pointer
    joint_positions_node__srv__GetJointDegrees_Response__rosidl_typesupport_introspection_c__assign_function__GetJointDegrees_Response__degrees,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers joint_positions_node__srv__GetJointDegrees_Response__rosidl_typesupport_introspection_c__GetJointDegrees_Response_message_members = {
  "joint_positions_node__srv",  // message namespace
  "GetJointDegrees_Response",  // message name
  1,  // number of fields
  sizeof(joint_positions_node__srv__GetJointDegrees_Response),
  joint_positions_node__srv__GetJointDegrees_Response__rosidl_typesupport_introspection_c__GetJointDegrees_Response_message_member_array,  // message members
  joint_positions_node__srv__GetJointDegrees_Response__rosidl_typesupport_introspection_c__GetJointDegrees_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  joint_positions_node__srv__GetJointDegrees_Response__rosidl_typesupport_introspection_c__GetJointDegrees_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t joint_positions_node__srv__GetJointDegrees_Response__rosidl_typesupport_introspection_c__GetJointDegrees_Response_message_type_support_handle = {
  0,
  &joint_positions_node__srv__GetJointDegrees_Response__rosidl_typesupport_introspection_c__GetJointDegrees_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_joint_positions_node
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, joint_positions_node, srv, GetJointDegrees_Response)() {
  if (!joint_positions_node__srv__GetJointDegrees_Response__rosidl_typesupport_introspection_c__GetJointDegrees_Response_message_type_support_handle.typesupport_identifier) {
    joint_positions_node__srv__GetJointDegrees_Response__rosidl_typesupport_introspection_c__GetJointDegrees_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &joint_positions_node__srv__GetJointDegrees_Response__rosidl_typesupport_introspection_c__GetJointDegrees_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "joint_positions_node/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "joint_positions_node/srv/detail/get_joint_degrees__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers joint_positions_node__srv__detail__get_joint_degrees__rosidl_typesupport_introspection_c__GetJointDegrees_service_members = {
  "joint_positions_node__srv",  // service namespace
  "GetJointDegrees",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // joint_positions_node__srv__detail__get_joint_degrees__rosidl_typesupport_introspection_c__GetJointDegrees_Request_message_type_support_handle,
  NULL  // response message
  // joint_positions_node__srv__detail__get_joint_degrees__rosidl_typesupport_introspection_c__GetJointDegrees_Response_message_type_support_handle
};

static rosidl_service_type_support_t joint_positions_node__srv__detail__get_joint_degrees__rosidl_typesupport_introspection_c__GetJointDegrees_service_type_support_handle = {
  0,
  &joint_positions_node__srv__detail__get_joint_degrees__rosidl_typesupport_introspection_c__GetJointDegrees_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, joint_positions_node, srv, GetJointDegrees_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, joint_positions_node, srv, GetJointDegrees_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_joint_positions_node
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, joint_positions_node, srv, GetJointDegrees)() {
  if (!joint_positions_node__srv__detail__get_joint_degrees__rosidl_typesupport_introspection_c__GetJointDegrees_service_type_support_handle.typesupport_identifier) {
    joint_positions_node__srv__detail__get_joint_degrees__rosidl_typesupport_introspection_c__GetJointDegrees_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)joint_positions_node__srv__detail__get_joint_degrees__rosidl_typesupport_introspection_c__GetJointDegrees_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, joint_positions_node, srv, GetJointDegrees_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, joint_positions_node, srv, GetJointDegrees_Response)()->data;
  }

  return &joint_positions_node__srv__detail__get_joint_degrees__rosidl_typesupport_introspection_c__GetJointDegrees_service_type_support_handle;
}
