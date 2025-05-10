// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from joint_positions_node:msg/SetJointDegrees.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "joint_positions_node/msg/detail/set_joint_degrees__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace joint_positions_node
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void SetJointDegrees_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) joint_positions_node::msg::SetJointDegrees(_init);
}

void SetJointDegrees_fini_function(void * message_memory)
{
  auto typed_message = static_cast<joint_positions_node::msg::SetJointDegrees *>(message_memory);
  typed_message->~SetJointDegrees();
}

size_t size_function__SetJointDegrees__degrees(const void * untyped_member)
{
  (void)untyped_member;
  return 2;
}

const void * get_const_function__SetJointDegrees__degrees(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<double, 2> *>(untyped_member);
  return &member[index];
}

void * get_function__SetJointDegrees__degrees(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<double, 2> *>(untyped_member);
  return &member[index];
}

void fetch_function__SetJointDegrees__degrees(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__SetJointDegrees__degrees(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__SetJointDegrees__degrees(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__SetJointDegrees__degrees(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember SetJointDegrees_message_member_array[1] = {
  {
    "degrees",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    2,  // array size
    false,  // is upper bound
    offsetof(joint_positions_node::msg::SetJointDegrees, degrees),  // bytes offset in struct
    nullptr,  // default value
    size_function__SetJointDegrees__degrees,  // size() function pointer
    get_const_function__SetJointDegrees__degrees,  // get_const(index) function pointer
    get_function__SetJointDegrees__degrees,  // get(index) function pointer
    fetch_function__SetJointDegrees__degrees,  // fetch(index, &value) function pointer
    assign_function__SetJointDegrees__degrees,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers SetJointDegrees_message_members = {
  "joint_positions_node::msg",  // message namespace
  "SetJointDegrees",  // message name
  1,  // number of fields
  sizeof(joint_positions_node::msg::SetJointDegrees),
  SetJointDegrees_message_member_array,  // message members
  SetJointDegrees_init_function,  // function to initialize message memory (memory has to be allocated)
  SetJointDegrees_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t SetJointDegrees_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &SetJointDegrees_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace joint_positions_node


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<joint_positions_node::msg::SetJointDegrees>()
{
  return &::joint_positions_node::msg::rosidl_typesupport_introspection_cpp::SetJointDegrees_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, joint_positions_node, msg, SetJointDegrees)() {
  return &::joint_positions_node::msg::rosidl_typesupport_introspection_cpp::SetJointDegrees_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
