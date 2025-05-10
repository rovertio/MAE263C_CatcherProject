// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from joint_positions_node:msg/SetJointDegrees.idl
// generated code does not contain a copyright notice

#ifndef JOINT_POSITIONS_NODE__MSG__DETAIL__SET_JOINT_DEGREES__BUILDER_HPP_
#define JOINT_POSITIONS_NODE__MSG__DETAIL__SET_JOINT_DEGREES__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "joint_positions_node/msg/detail/set_joint_degrees__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace joint_positions_node
{

namespace msg
{

namespace builder
{

class Init_SetJointDegrees_degrees
{
public:
  Init_SetJointDegrees_degrees()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::joint_positions_node::msg::SetJointDegrees degrees(::joint_positions_node::msg::SetJointDegrees::_degrees_type arg)
  {
    msg_.degrees = std::move(arg);
    return std::move(msg_);
  }

private:
  ::joint_positions_node::msg::SetJointDegrees msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::joint_positions_node::msg::SetJointDegrees>()
{
  return joint_positions_node::msg::builder::Init_SetJointDegrees_degrees();
}

}  // namespace joint_positions_node

#endif  // JOINT_POSITIONS_NODE__MSG__DETAIL__SET_JOINT_DEGREES__BUILDER_HPP_
