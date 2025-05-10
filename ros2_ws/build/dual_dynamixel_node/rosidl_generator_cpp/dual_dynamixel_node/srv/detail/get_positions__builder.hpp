// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from dual_dynamixel_node:srv/GetPositions.idl
// generated code does not contain a copyright notice

#ifndef DUAL_DYNAMIXEL_NODE__SRV__DETAIL__GET_POSITIONS__BUILDER_HPP_
#define DUAL_DYNAMIXEL_NODE__SRV__DETAIL__GET_POSITIONS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "dual_dynamixel_node/srv/detail/get_positions__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace dual_dynamixel_node
{

namespace srv
{

namespace builder
{

class Init_GetPositions_Request_id
{
public:
  Init_GetPositions_Request_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::dual_dynamixel_node::srv::GetPositions_Request id(::dual_dynamixel_node::srv::GetPositions_Request::_id_type arg)
  {
    msg_.id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::dual_dynamixel_node::srv::GetPositions_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::dual_dynamixel_node::srv::GetPositions_Request>()
{
  return dual_dynamixel_node::srv::builder::Init_GetPositions_Request_id();
}

}  // namespace dual_dynamixel_node


namespace dual_dynamixel_node
{

namespace srv
{

namespace builder
{

class Init_GetPositions_Response_position
{
public:
  Init_GetPositions_Response_position()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::dual_dynamixel_node::srv::GetPositions_Response position(::dual_dynamixel_node::srv::GetPositions_Response::_position_type arg)
  {
    msg_.position = std::move(arg);
    return std::move(msg_);
  }

private:
  ::dual_dynamixel_node::srv::GetPositions_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::dual_dynamixel_node::srv::GetPositions_Response>()
{
  return dual_dynamixel_node::srv::builder::Init_GetPositions_Response_position();
}

}  // namespace dual_dynamixel_node

#endif  // DUAL_DYNAMIXEL_NODE__SRV__DETAIL__GET_POSITIONS__BUILDER_HPP_
