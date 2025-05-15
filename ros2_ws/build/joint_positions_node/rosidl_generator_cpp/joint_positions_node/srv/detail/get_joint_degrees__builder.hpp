// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from joint_positions_node:srv/GetJointDegrees.idl
// generated code does not contain a copyright notice

#ifndef JOINT_POSITIONS_NODE__SRV__DETAIL__GET_JOINT_DEGREES__BUILDER_HPP_
#define JOINT_POSITIONS_NODE__SRV__DETAIL__GET_JOINT_DEGREES__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "joint_positions_node/srv/detail/get_joint_degrees__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace joint_positions_node
{

namespace srv
{

namespace builder
{

class Init_GetJointDegrees_Request_dummy
{
public:
  Init_GetJointDegrees_Request_dummy()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::joint_positions_node::srv::GetJointDegrees_Request dummy(::joint_positions_node::srv::GetJointDegrees_Request::_dummy_type arg)
  {
    msg_.dummy = std::move(arg);
    return std::move(msg_);
  }

private:
  ::joint_positions_node::srv::GetJointDegrees_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::joint_positions_node::srv::GetJointDegrees_Request>()
{
  return joint_positions_node::srv::builder::Init_GetJointDegrees_Request_dummy();
}

}  // namespace joint_positions_node


namespace joint_positions_node
{

namespace srv
{

namespace builder
{

class Init_GetJointDegrees_Response_degrees
{
public:
  Init_GetJointDegrees_Response_degrees()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::joint_positions_node::srv::GetJointDegrees_Response degrees(::joint_positions_node::srv::GetJointDegrees_Response::_degrees_type arg)
  {
    msg_.degrees = std::move(arg);
    return std::move(msg_);
  }

private:
  ::joint_positions_node::srv::GetJointDegrees_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::joint_positions_node::srv::GetJointDegrees_Response>()
{
  return joint_positions_node::srv::builder::Init_GetJointDegrees_Response_degrees();
}

}  // namespace joint_positions_node

#endif  // JOINT_POSITIONS_NODE__SRV__DETAIL__GET_JOINT_DEGREES__BUILDER_HPP_
