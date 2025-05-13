// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from dual_dynamixel_node:msg/SetPositions.idl
// generated code does not contain a copyright notice

#ifndef DUAL_DYNAMIXEL_NODE__MSG__DETAIL__SET_POSITIONS__BUILDER_HPP_
#define DUAL_DYNAMIXEL_NODE__MSG__DETAIL__SET_POSITIONS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "dual_dynamixel_node/msg/detail/set_positions__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace dual_dynamixel_node
{

namespace msg
{

namespace builder
{

class Init_SetPositions_position
{
public:
  explicit Init_SetPositions_position(::dual_dynamixel_node::msg::SetPositions & msg)
  : msg_(msg)
  {}
  ::dual_dynamixel_node::msg::SetPositions position(::dual_dynamixel_node::msg::SetPositions::_position_type arg)
  {
    msg_.position = std::move(arg);
    return std::move(msg_);
  }

private:
  ::dual_dynamixel_node::msg::SetPositions msg_;
};

class Init_SetPositions_id
{
public:
  Init_SetPositions_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SetPositions_position id(::dual_dynamixel_node::msg::SetPositions::_id_type arg)
  {
    msg_.id = std::move(arg);
    return Init_SetPositions_position(msg_);
  }

private:
  ::dual_dynamixel_node::msg::SetPositions msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::dual_dynamixel_node::msg::SetPositions>()
{
  return dual_dynamixel_node::msg::builder::Init_SetPositions_id();
}

}  // namespace dual_dynamixel_node

#endif  // DUAL_DYNAMIXEL_NODE__MSG__DETAIL__SET_POSITIONS__BUILDER_HPP_
