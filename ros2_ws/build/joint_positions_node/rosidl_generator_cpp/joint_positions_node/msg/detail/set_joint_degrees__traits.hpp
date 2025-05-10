// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from joint_positions_node:msg/SetJointDegrees.idl
// generated code does not contain a copyright notice

#ifndef JOINT_POSITIONS_NODE__MSG__DETAIL__SET_JOINT_DEGREES__TRAITS_HPP_
#define JOINT_POSITIONS_NODE__MSG__DETAIL__SET_JOINT_DEGREES__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "joint_positions_node/msg/detail/set_joint_degrees__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace joint_positions_node
{

namespace msg
{

inline void to_flow_style_yaml(
  const SetJointDegrees & msg,
  std::ostream & out)
{
  out << "{";
  // member: degrees
  {
    if (msg.degrees.size() == 0) {
      out << "degrees: []";
    } else {
      out << "degrees: [";
      size_t pending_items = msg.degrees.size();
      for (auto item : msg.degrees) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SetJointDegrees & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: degrees
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.degrees.size() == 0) {
      out << "degrees: []\n";
    } else {
      out << "degrees:\n";
      for (auto item : msg.degrees) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SetJointDegrees & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace joint_positions_node

namespace rosidl_generator_traits
{

[[deprecated("use joint_positions_node::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const joint_positions_node::msg::SetJointDegrees & msg,
  std::ostream & out, size_t indentation = 0)
{
  joint_positions_node::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use joint_positions_node::msg::to_yaml() instead")]]
inline std::string to_yaml(const joint_positions_node::msg::SetJointDegrees & msg)
{
  return joint_positions_node::msg::to_yaml(msg);
}

template<>
inline const char * data_type<joint_positions_node::msg::SetJointDegrees>()
{
  return "joint_positions_node::msg::SetJointDegrees";
}

template<>
inline const char * name<joint_positions_node::msg::SetJointDegrees>()
{
  return "joint_positions_node/msg/SetJointDegrees";
}

template<>
struct has_fixed_size<joint_positions_node::msg::SetJointDegrees>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<joint_positions_node::msg::SetJointDegrees>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<joint_positions_node::msg::SetJointDegrees>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // JOINT_POSITIONS_NODE__MSG__DETAIL__SET_JOINT_DEGREES__TRAITS_HPP_
