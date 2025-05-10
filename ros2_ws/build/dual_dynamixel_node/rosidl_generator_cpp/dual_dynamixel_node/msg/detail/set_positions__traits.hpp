// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from dual_dynamixel_node:msg/SetPositions.idl
// generated code does not contain a copyright notice

#ifndef DUAL_DYNAMIXEL_NODE__MSG__DETAIL__SET_POSITIONS__TRAITS_HPP_
#define DUAL_DYNAMIXEL_NODE__MSG__DETAIL__SET_POSITIONS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "dual_dynamixel_node/msg/detail/set_positions__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace dual_dynamixel_node
{

namespace msg
{

inline void to_flow_style_yaml(
  const SetPositions & msg,
  std::ostream & out)
{
  out << "{";
  // member: id
  {
    if (msg.id.size() == 0) {
      out << "id: []";
    } else {
      out << "id: [";
      size_t pending_items = msg.id.size();
      for (auto item : msg.id) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: position
  {
    if (msg.position.size() == 0) {
      out << "position: []";
    } else {
      out << "position: [";
      size_t pending_items = msg.position.size();
      for (auto item : msg.position) {
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
  const SetPositions & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.id.size() == 0) {
      out << "id: []\n";
    } else {
      out << "id:\n";
      for (auto item : msg.id) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: position
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.position.size() == 0) {
      out << "position: []\n";
    } else {
      out << "position:\n";
      for (auto item : msg.position) {
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

inline std::string to_yaml(const SetPositions & msg, bool use_flow_style = false)
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

}  // namespace dual_dynamixel_node

namespace rosidl_generator_traits
{

[[deprecated("use dual_dynamixel_node::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const dual_dynamixel_node::msg::SetPositions & msg,
  std::ostream & out, size_t indentation = 0)
{
  dual_dynamixel_node::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use dual_dynamixel_node::msg::to_yaml() instead")]]
inline std::string to_yaml(const dual_dynamixel_node::msg::SetPositions & msg)
{
  return dual_dynamixel_node::msg::to_yaml(msg);
}

template<>
inline const char * data_type<dual_dynamixel_node::msg::SetPositions>()
{
  return "dual_dynamixel_node::msg::SetPositions";
}

template<>
inline const char * name<dual_dynamixel_node::msg::SetPositions>()
{
  return "dual_dynamixel_node/msg/SetPositions";
}

template<>
struct has_fixed_size<dual_dynamixel_node::msg::SetPositions>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<dual_dynamixel_node::msg::SetPositions>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<dual_dynamixel_node::msg::SetPositions>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // DUAL_DYNAMIXEL_NODE__MSG__DETAIL__SET_POSITIONS__TRAITS_HPP_
