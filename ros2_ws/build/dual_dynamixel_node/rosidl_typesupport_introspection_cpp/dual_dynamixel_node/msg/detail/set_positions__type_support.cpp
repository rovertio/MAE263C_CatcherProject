// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from dual_dynamixel_node:msg/SetPositions.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "dual_dynamixel_node/msg/detail/set_positions__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace dual_dynamixel_node
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void SetPositions_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) dual_dynamixel_node::msg::SetPositions(_init);
}

void SetPositions_fini_function(void * message_memory)
{
  auto typed_message = static_cast<dual_dynamixel_node::msg::SetPositions *>(message_memory);
  typed_message->~SetPositions();
}

size_t size_function__SetPositions__id(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<uint8_t> *>(untyped_member);
  return member->size();
}

const void * get_const_function__SetPositions__id(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<uint8_t> *>(untyped_member);
  return &member[index];
}

void * get_function__SetPositions__id(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<uint8_t> *>(untyped_member);
  return &member[index];
}

void fetch_function__SetPositions__id(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const uint8_t *>(
    get_const_function__SetPositions__id(untyped_member, index));
  auto & value = *reinterpret_cast<uint8_t *>(untyped_value);
  value = item;
}

void assign_function__SetPositions__id(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<uint8_t *>(
    get_function__SetPositions__id(untyped_member, index));
  const auto & value = *reinterpret_cast<const uint8_t *>(untyped_value);
  item = value;
}

void resize_function__SetPositions__id(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<uint8_t> *>(untyped_member);
  member->resize(size);
}

size_t size_function__SetPositions__position(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<int32_t> *>(untyped_member);
  return member->size();
}

const void * get_const_function__SetPositions__position(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<int32_t> *>(untyped_member);
  return &member[index];
}

void * get_function__SetPositions__position(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<int32_t> *>(untyped_member);
  return &member[index];
}

void fetch_function__SetPositions__position(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const int32_t *>(
    get_const_function__SetPositions__position(untyped_member, index));
  auto & value = *reinterpret_cast<int32_t *>(untyped_value);
  value = item;
}

void assign_function__SetPositions__position(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<int32_t *>(
    get_function__SetPositions__position(untyped_member, index));
  const auto & value = *reinterpret_cast<const int32_t *>(untyped_value);
  item = value;
}

void resize_function__SetPositions__position(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<int32_t> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember SetPositions_message_member_array[2] = {
  {
    "id",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dual_dynamixel_node::msg::SetPositions, id),  // bytes offset in struct
    nullptr,  // default value
    size_function__SetPositions__id,  // size() function pointer
    get_const_function__SetPositions__id,  // get_const(index) function pointer
    get_function__SetPositions__id,  // get(index) function pointer
    fetch_function__SetPositions__id,  // fetch(index, &value) function pointer
    assign_function__SetPositions__id,  // assign(index, value) function pointer
    resize_function__SetPositions__id  // resize(index) function pointer
  },
  {
    "position",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dual_dynamixel_node::msg::SetPositions, position),  // bytes offset in struct
    nullptr,  // default value
    size_function__SetPositions__position,  // size() function pointer
    get_const_function__SetPositions__position,  // get_const(index) function pointer
    get_function__SetPositions__position,  // get(index) function pointer
    fetch_function__SetPositions__position,  // fetch(index, &value) function pointer
    assign_function__SetPositions__position,  // assign(index, value) function pointer
    resize_function__SetPositions__position  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers SetPositions_message_members = {
  "dual_dynamixel_node::msg",  // message namespace
  "SetPositions",  // message name
  2,  // number of fields
  sizeof(dual_dynamixel_node::msg::SetPositions),
  SetPositions_message_member_array,  // message members
  SetPositions_init_function,  // function to initialize message memory (memory has to be allocated)
  SetPositions_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t SetPositions_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &SetPositions_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace dual_dynamixel_node


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<dual_dynamixel_node::msg::SetPositions>()
{
  return &::dual_dynamixel_node::msg::rosidl_typesupport_introspection_cpp::SetPositions_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, dual_dynamixel_node, msg, SetPositions)() {
  return &::dual_dynamixel_node::msg::rosidl_typesupport_introspection_cpp::SetPositions_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
