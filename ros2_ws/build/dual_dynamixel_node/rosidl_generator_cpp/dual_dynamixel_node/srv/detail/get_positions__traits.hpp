// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from dual_dynamixel_node:srv/GetPositions.idl
// generated code does not contain a copyright notice

#ifndef DUAL_DYNAMIXEL_NODE__SRV__DETAIL__GET_POSITIONS__TRAITS_HPP_
#define DUAL_DYNAMIXEL_NODE__SRV__DETAIL__GET_POSITIONS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "dual_dynamixel_node/srv/detail/get_positions__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace dual_dynamixel_node
{

namespace srv
{

inline void to_flow_style_yaml(
  const GetPositions_Request & msg,
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
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GetPositions_Request & msg,
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
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GetPositions_Request & msg, bool use_flow_style = false)
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

}  // namespace dual_dynamixel_node

namespace rosidl_generator_traits
{

[[deprecated("use dual_dynamixel_node::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const dual_dynamixel_node::srv::GetPositions_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  dual_dynamixel_node::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use dual_dynamixel_node::srv::to_yaml() instead")]]
inline std::string to_yaml(const dual_dynamixel_node::srv::GetPositions_Request & msg)
{
  return dual_dynamixel_node::srv::to_yaml(msg);
}

template<>
inline const char * data_type<dual_dynamixel_node::srv::GetPositions_Request>()
{
  return "dual_dynamixel_node::srv::GetPositions_Request";
}

template<>
inline const char * name<dual_dynamixel_node::srv::GetPositions_Request>()
{
  return "dual_dynamixel_node/srv/GetPositions_Request";
}

template<>
struct has_fixed_size<dual_dynamixel_node::srv::GetPositions_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<dual_dynamixel_node::srv::GetPositions_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<dual_dynamixel_node::srv::GetPositions_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace dual_dynamixel_node
{

namespace srv
{

inline void to_flow_style_yaml(
  const GetPositions_Response & msg,
  std::ostream & out)
{
  out << "{";
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
  const GetPositions_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
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

inline std::string to_yaml(const GetPositions_Response & msg, bool use_flow_style = false)
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

}  // namespace dual_dynamixel_node

namespace rosidl_generator_traits
{

[[deprecated("use dual_dynamixel_node::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const dual_dynamixel_node::srv::GetPositions_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  dual_dynamixel_node::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use dual_dynamixel_node::srv::to_yaml() instead")]]
inline std::string to_yaml(const dual_dynamixel_node::srv::GetPositions_Response & msg)
{
  return dual_dynamixel_node::srv::to_yaml(msg);
}

template<>
inline const char * data_type<dual_dynamixel_node::srv::GetPositions_Response>()
{
  return "dual_dynamixel_node::srv::GetPositions_Response";
}

template<>
inline const char * name<dual_dynamixel_node::srv::GetPositions_Response>()
{
  return "dual_dynamixel_node/srv/GetPositions_Response";
}

template<>
struct has_fixed_size<dual_dynamixel_node::srv::GetPositions_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<dual_dynamixel_node::srv::GetPositions_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<dual_dynamixel_node::srv::GetPositions_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<dual_dynamixel_node::srv::GetPositions>()
{
  return "dual_dynamixel_node::srv::GetPositions";
}

template<>
inline const char * name<dual_dynamixel_node::srv::GetPositions>()
{
  return "dual_dynamixel_node/srv/GetPositions";
}

template<>
struct has_fixed_size<dual_dynamixel_node::srv::GetPositions>
  : std::integral_constant<
    bool,
    has_fixed_size<dual_dynamixel_node::srv::GetPositions_Request>::value &&
    has_fixed_size<dual_dynamixel_node::srv::GetPositions_Response>::value
  >
{
};

template<>
struct has_bounded_size<dual_dynamixel_node::srv::GetPositions>
  : std::integral_constant<
    bool,
    has_bounded_size<dual_dynamixel_node::srv::GetPositions_Request>::value &&
    has_bounded_size<dual_dynamixel_node::srv::GetPositions_Response>::value
  >
{
};

template<>
struct is_service<dual_dynamixel_node::srv::GetPositions>
  : std::true_type
{
};

template<>
struct is_service_request<dual_dynamixel_node::srv::GetPositions_Request>
  : std::true_type
{
};

template<>
struct is_service_response<dual_dynamixel_node::srv::GetPositions_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // DUAL_DYNAMIXEL_NODE__SRV__DETAIL__GET_POSITIONS__TRAITS_HPP_
