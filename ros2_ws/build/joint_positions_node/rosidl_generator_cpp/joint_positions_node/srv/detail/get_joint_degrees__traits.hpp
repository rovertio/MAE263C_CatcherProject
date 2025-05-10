// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from joint_positions_node:srv/GetJointDegrees.idl
// generated code does not contain a copyright notice

#ifndef JOINT_POSITIONS_NODE__SRV__DETAIL__GET_JOINT_DEGREES__TRAITS_HPP_
#define JOINT_POSITIONS_NODE__SRV__DETAIL__GET_JOINT_DEGREES__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "joint_positions_node/srv/detail/get_joint_degrees__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace joint_positions_node
{

namespace srv
{

inline void to_flow_style_yaml(
  const GetJointDegrees_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: dummy
  {
    out << "dummy: ";
    rosidl_generator_traits::value_to_yaml(msg.dummy, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GetJointDegrees_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: dummy
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "dummy: ";
    rosidl_generator_traits::value_to_yaml(msg.dummy, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GetJointDegrees_Request & msg, bool use_flow_style = false)
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

}  // namespace joint_positions_node

namespace rosidl_generator_traits
{

[[deprecated("use joint_positions_node::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const joint_positions_node::srv::GetJointDegrees_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  joint_positions_node::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use joint_positions_node::srv::to_yaml() instead")]]
inline std::string to_yaml(const joint_positions_node::srv::GetJointDegrees_Request & msg)
{
  return joint_positions_node::srv::to_yaml(msg);
}

template<>
inline const char * data_type<joint_positions_node::srv::GetJointDegrees_Request>()
{
  return "joint_positions_node::srv::GetJointDegrees_Request";
}

template<>
inline const char * name<joint_positions_node::srv::GetJointDegrees_Request>()
{
  return "joint_positions_node/srv/GetJointDegrees_Request";
}

template<>
struct has_fixed_size<joint_positions_node::srv::GetJointDegrees_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<joint_positions_node::srv::GetJointDegrees_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<joint_positions_node::srv::GetJointDegrees_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace joint_positions_node
{

namespace srv
{

inline void to_flow_style_yaml(
  const GetJointDegrees_Response & msg,
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
  const GetJointDegrees_Response & msg,
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

inline std::string to_yaml(const GetJointDegrees_Response & msg, bool use_flow_style = false)
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

}  // namespace joint_positions_node

namespace rosidl_generator_traits
{

[[deprecated("use joint_positions_node::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const joint_positions_node::srv::GetJointDegrees_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  joint_positions_node::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use joint_positions_node::srv::to_yaml() instead")]]
inline std::string to_yaml(const joint_positions_node::srv::GetJointDegrees_Response & msg)
{
  return joint_positions_node::srv::to_yaml(msg);
}

template<>
inline const char * data_type<joint_positions_node::srv::GetJointDegrees_Response>()
{
  return "joint_positions_node::srv::GetJointDegrees_Response";
}

template<>
inline const char * name<joint_positions_node::srv::GetJointDegrees_Response>()
{
  return "joint_positions_node/srv/GetJointDegrees_Response";
}

template<>
struct has_fixed_size<joint_positions_node::srv::GetJointDegrees_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<joint_positions_node::srv::GetJointDegrees_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<joint_positions_node::srv::GetJointDegrees_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<joint_positions_node::srv::GetJointDegrees>()
{
  return "joint_positions_node::srv::GetJointDegrees";
}

template<>
inline const char * name<joint_positions_node::srv::GetJointDegrees>()
{
  return "joint_positions_node/srv/GetJointDegrees";
}

template<>
struct has_fixed_size<joint_positions_node::srv::GetJointDegrees>
  : std::integral_constant<
    bool,
    has_fixed_size<joint_positions_node::srv::GetJointDegrees_Request>::value &&
    has_fixed_size<joint_positions_node::srv::GetJointDegrees_Response>::value
  >
{
};

template<>
struct has_bounded_size<joint_positions_node::srv::GetJointDegrees>
  : std::integral_constant<
    bool,
    has_bounded_size<joint_positions_node::srv::GetJointDegrees_Request>::value &&
    has_bounded_size<joint_positions_node::srv::GetJointDegrees_Response>::value
  >
{
};

template<>
struct is_service<joint_positions_node::srv::GetJointDegrees>
  : std::true_type
{
};

template<>
struct is_service_request<joint_positions_node::srv::GetJointDegrees_Request>
  : std::true_type
{
};

template<>
struct is_service_response<joint_positions_node::srv::GetJointDegrees_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // JOINT_POSITIONS_NODE__SRV__DETAIL__GET_JOINT_DEGREES__TRAITS_HPP_
