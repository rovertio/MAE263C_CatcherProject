// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from inverse_kinematics_node:msg/SetXY.idl
// generated code does not contain a copyright notice

#ifndef INVERSE_KINEMATICS_NODE__MSG__DETAIL__SET_XY__STRUCT_HPP_
#define INVERSE_KINEMATICS_NODE__MSG__DETAIL__SET_XY__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__inverse_kinematics_node__msg__SetXY __attribute__((deprecated))
#else
# define DEPRECATED__inverse_kinematics_node__msg__SetXY __declspec(deprecated)
#endif

namespace inverse_kinematics_node
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct SetXY_
{
  using Type = SetXY_<ContainerAllocator>;

  explicit SetXY_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->x = 0.0;
      this->y = 0.0;
    }
  }

  explicit SetXY_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->x = 0.0;
      this->y = 0.0;
    }
  }

  // field types and members
  using _x_type =
    double;
  _x_type x;
  using _y_type =
    double;
  _y_type y;

  // setters for named parameter idiom
  Type & set__x(
    const double & _arg)
  {
    this->x = _arg;
    return *this;
  }
  Type & set__y(
    const double & _arg)
  {
    this->y = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    inverse_kinematics_node::msg::SetXY_<ContainerAllocator> *;
  using ConstRawPtr =
    const inverse_kinematics_node::msg::SetXY_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<inverse_kinematics_node::msg::SetXY_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<inverse_kinematics_node::msg::SetXY_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      inverse_kinematics_node::msg::SetXY_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<inverse_kinematics_node::msg::SetXY_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      inverse_kinematics_node::msg::SetXY_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<inverse_kinematics_node::msg::SetXY_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<inverse_kinematics_node::msg::SetXY_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<inverse_kinematics_node::msg::SetXY_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__inverse_kinematics_node__msg__SetXY
    std::shared_ptr<inverse_kinematics_node::msg::SetXY_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__inverse_kinematics_node__msg__SetXY
    std::shared_ptr<inverse_kinematics_node::msg::SetXY_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SetXY_ & other) const
  {
    if (this->x != other.x) {
      return false;
    }
    if (this->y != other.y) {
      return false;
    }
    return true;
  }
  bool operator!=(const SetXY_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SetXY_

// alias to use template instance with default allocator
using SetXY =
  inverse_kinematics_node::msg::SetXY_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace inverse_kinematics_node

#endif  // INVERSE_KINEMATICS_NODE__MSG__DETAIL__SET_XY__STRUCT_HPP_
