// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from inverse_kinematics_node:msg/SetXY.idl
// generated code does not contain a copyright notice

#ifndef INVERSE_KINEMATICS_NODE__MSG__DETAIL__SET_XY__TRAITS_HPP_
#define INVERSE_KINEMATICS_NODE__MSG__DETAIL__SET_XY__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "inverse_kinematics_node/msg/detail/set_xy__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace inverse_kinematics_node
{

namespace msg
{

inline void to_flow_style_yaml(
  const SetXY & msg,
  std::ostream & out)
{
  out << "{";
  // member: x
  {
    out << "x: ";
    rosidl_generator_traits::value_to_yaml(msg.x, out);
    out << ", ";
  }

  // member: y
  {
    out << "y: ";
    rosidl_generator_traits::value_to_yaml(msg.y, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SetXY & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "x: ";
    rosidl_generator_traits::value_to_yaml(msg.x, out);
    out << "\n";
  }

  // member: y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "y: ";
    rosidl_generator_traits::value_to_yaml(msg.y, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SetXY & msg, bool use_flow_style = false)
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

}  // namespace inverse_kinematics_node

namespace rosidl_generator_traits
{

[[deprecated("use inverse_kinematics_node::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const inverse_kinematics_node::msg::SetXY & msg,
  std::ostream & out, size_t indentation = 0)
{
  inverse_kinematics_node::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use inverse_kinematics_node::msg::to_yaml() instead")]]
inline std::string to_yaml(const inverse_kinematics_node::msg::SetXY & msg)
{
  return inverse_kinematics_node::msg::to_yaml(msg);
}

template<>
inline const char * data_type<inverse_kinematics_node::msg::SetXY>()
{
  return "inverse_kinematics_node::msg::SetXY";
}

template<>
inline const char * name<inverse_kinematics_node::msg::SetXY>()
{
  return "inverse_kinematics_node/msg/SetXY";
}

template<>
struct has_fixed_size<inverse_kinematics_node::msg::SetXY>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<inverse_kinematics_node::msg::SetXY>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<inverse_kinematics_node::msg::SetXY>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // INVERSE_KINEMATICS_NODE__MSG__DETAIL__SET_XY__TRAITS_HPP_
