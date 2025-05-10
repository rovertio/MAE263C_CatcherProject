// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from inverse_kinematics_node:srv/GetXY.idl
// generated code does not contain a copyright notice

#ifndef INVERSE_KINEMATICS_NODE__SRV__DETAIL__GET_XY__BUILDER_HPP_
#define INVERSE_KINEMATICS_NODE__SRV__DETAIL__GET_XY__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "inverse_kinematics_node/srv/detail/get_xy__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace inverse_kinematics_node
{

namespace srv
{


}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::inverse_kinematics_node::srv::GetXY_Request>()
{
  return ::inverse_kinematics_node::srv::GetXY_Request(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace inverse_kinematics_node


namespace inverse_kinematics_node
{

namespace srv
{

namespace builder
{

class Init_GetXY_Response_y
{
public:
  explicit Init_GetXY_Response_y(::inverse_kinematics_node::srv::GetXY_Response & msg)
  : msg_(msg)
  {}
  ::inverse_kinematics_node::srv::GetXY_Response y(::inverse_kinematics_node::srv::GetXY_Response::_y_type arg)
  {
    msg_.y = std::move(arg);
    return std::move(msg_);
  }

private:
  ::inverse_kinematics_node::srv::GetXY_Response msg_;
};

class Init_GetXY_Response_x
{
public:
  Init_GetXY_Response_x()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GetXY_Response_y x(::inverse_kinematics_node::srv::GetXY_Response::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_GetXY_Response_y(msg_);
  }

private:
  ::inverse_kinematics_node::srv::GetXY_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::inverse_kinematics_node::srv::GetXY_Response>()
{
  return inverse_kinematics_node::srv::builder::Init_GetXY_Response_x();
}

}  // namespace inverse_kinematics_node

#endif  // INVERSE_KINEMATICS_NODE__SRV__DETAIL__GET_XY__BUILDER_HPP_
