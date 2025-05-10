// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from inverse_kinematics_node:srv/GetXY.idl
// generated code does not contain a copyright notice

#ifndef INVERSE_KINEMATICS_NODE__SRV__DETAIL__GET_XY__TRAITS_HPP_
#define INVERSE_KINEMATICS_NODE__SRV__DETAIL__GET_XY__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "inverse_kinematics_node/srv/detail/get_xy__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace inverse_kinematics_node
{

namespace srv
{

inline void to_flow_style_yaml(
  const GetXY_Request & msg,
  std::ostream & out)
{
  (void)msg;
  out << "null";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GetXY_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  (void)msg;
  (void)indentation;
  out << "null\n";
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GetXY_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace inverse_kinematics_node

namespace rosidl_generator_traits
{

[[deprecated("use inverse_kinematics_node::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const inverse_kinematics_node::srv::GetXY_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  inverse_kinematics_node::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use inverse_kinematics_node::srv::to_yaml() instead")]]
inline std::string to_yaml(const inverse_kinematics_node::srv::GetXY_Request & msg)
{
  return inverse_kinematics_node::srv::to_yaml(msg);
}

template<>
inline const char * data_type<inverse_kinematics_node::srv::GetXY_Request>()
{
  return "inverse_kinematics_node::srv::GetXY_Request";
}

template<>
inline const char * name<inverse_kinematics_node::srv::GetXY_Request>()
{
  return "inverse_kinematics_node/srv/GetXY_Request";
}

template<>
struct has_fixed_size<inverse_kinematics_node::srv::GetXY_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<inverse_kinematics_node::srv::GetXY_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<inverse_kinematics_node::srv::GetXY_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace inverse_kinematics_node
{

namespace srv
{

inline void to_flow_style_yaml(
  const GetXY_Response & msg,
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
  const GetXY_Response & msg,
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

inline std::string to_yaml(const GetXY_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace inverse_kinematics_node

namespace rosidl_generator_traits
{

[[deprecated("use inverse_kinematics_node::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const inverse_kinematics_node::srv::GetXY_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  inverse_kinematics_node::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use inverse_kinematics_node::srv::to_yaml() instead")]]
inline std::string to_yaml(const inverse_kinematics_node::srv::GetXY_Response & msg)
{
  return inverse_kinematics_node::srv::to_yaml(msg);
}

template<>
inline const char * data_type<inverse_kinematics_node::srv::GetXY_Response>()
{
  return "inverse_kinematics_node::srv::GetXY_Response";
}

template<>
inline const char * name<inverse_kinematics_node::srv::GetXY_Response>()
{
  return "inverse_kinematics_node/srv/GetXY_Response";
}

template<>
struct has_fixed_size<inverse_kinematics_node::srv::GetXY_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<inverse_kinematics_node::srv::GetXY_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<inverse_kinematics_node::srv::GetXY_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<inverse_kinematics_node::srv::GetXY>()
{
  return "inverse_kinematics_node::srv::GetXY";
}

template<>
inline const char * name<inverse_kinematics_node::srv::GetXY>()
{
  return "inverse_kinematics_node/srv/GetXY";
}

template<>
struct has_fixed_size<inverse_kinematics_node::srv::GetXY>
  : std::integral_constant<
    bool,
    has_fixed_size<inverse_kinematics_node::srv::GetXY_Request>::value &&
    has_fixed_size<inverse_kinematics_node::srv::GetXY_Response>::value
  >
{
};

template<>
struct has_bounded_size<inverse_kinematics_node::srv::GetXY>
  : std::integral_constant<
    bool,
    has_bounded_size<inverse_kinematics_node::srv::GetXY_Request>::value &&
    has_bounded_size<inverse_kinematics_node::srv::GetXY_Response>::value
  >
{
};

template<>
struct is_service<inverse_kinematics_node::srv::GetXY>
  : std::true_type
{
};

template<>
struct is_service_request<inverse_kinematics_node::srv::GetXY_Request>
  : std::true_type
{
};

template<>
struct is_service_response<inverse_kinematics_node::srv::GetXY_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // INVERSE_KINEMATICS_NODE__SRV__DETAIL__GET_XY__TRAITS_HPP_
