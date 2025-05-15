// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from inverse_kinematics_node:msg/SetXY.idl
// generated code does not contain a copyright notice

#ifndef INVERSE_KINEMATICS_NODE__MSG__DETAIL__SET_XY__BUILDER_HPP_
#define INVERSE_KINEMATICS_NODE__MSG__DETAIL__SET_XY__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "inverse_kinematics_node/msg/detail/set_xy__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace inverse_kinematics_node
{

namespace msg
{

namespace builder
{

class Init_SetXY_y
{
public:
  explicit Init_SetXY_y(::inverse_kinematics_node::msg::SetXY & msg)
  : msg_(msg)
  {}
  ::inverse_kinematics_node::msg::SetXY y(::inverse_kinematics_node::msg::SetXY::_y_type arg)
  {
    msg_.y = std::move(arg);
    return std::move(msg_);
  }

private:
  ::inverse_kinematics_node::msg::SetXY msg_;
};

class Init_SetXY_x
{
public:
  Init_SetXY_x()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SetXY_y x(::inverse_kinematics_node::msg::SetXY::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_SetXY_y(msg_);
  }

private:
  ::inverse_kinematics_node::msg::SetXY msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::inverse_kinematics_node::msg::SetXY>()
{
  return inverse_kinematics_node::msg::builder::Init_SetXY_x();
}

}  // namespace inverse_kinematics_node

#endif  // INVERSE_KINEMATICS_NODE__MSG__DETAIL__SET_XY__BUILDER_HPP_
