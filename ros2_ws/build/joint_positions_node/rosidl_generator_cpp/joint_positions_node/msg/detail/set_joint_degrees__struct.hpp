// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from joint_positions_node:msg/SetJointDegrees.idl
// generated code does not contain a copyright notice

#ifndef JOINT_POSITIONS_NODE__MSG__DETAIL__SET_JOINT_DEGREES__STRUCT_HPP_
#define JOINT_POSITIONS_NODE__MSG__DETAIL__SET_JOINT_DEGREES__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__joint_positions_node__msg__SetJointDegrees __attribute__((deprecated))
#else
# define DEPRECATED__joint_positions_node__msg__SetJointDegrees __declspec(deprecated)
#endif

namespace joint_positions_node
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct SetJointDegrees_
{
  using Type = SetJointDegrees_<ContainerAllocator>;

  explicit SetJointDegrees_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      std::fill<typename std::array<double, 2>::iterator, double>(this->degrees.begin(), this->degrees.end(), 0.0);
    }
  }

  explicit SetJointDegrees_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : degrees(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      std::fill<typename std::array<double, 2>::iterator, double>(this->degrees.begin(), this->degrees.end(), 0.0);
    }
  }

  // field types and members
  using _degrees_type =
    std::array<double, 2>;
  _degrees_type degrees;

  // setters for named parameter idiom
  Type & set__degrees(
    const std::array<double, 2> & _arg)
  {
    this->degrees = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    joint_positions_node::msg::SetJointDegrees_<ContainerAllocator> *;
  using ConstRawPtr =
    const joint_positions_node::msg::SetJointDegrees_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<joint_positions_node::msg::SetJointDegrees_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<joint_positions_node::msg::SetJointDegrees_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      joint_positions_node::msg::SetJointDegrees_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<joint_positions_node::msg::SetJointDegrees_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      joint_positions_node::msg::SetJointDegrees_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<joint_positions_node::msg::SetJointDegrees_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<joint_positions_node::msg::SetJointDegrees_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<joint_positions_node::msg::SetJointDegrees_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__joint_positions_node__msg__SetJointDegrees
    std::shared_ptr<joint_positions_node::msg::SetJointDegrees_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__joint_positions_node__msg__SetJointDegrees
    std::shared_ptr<joint_positions_node::msg::SetJointDegrees_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SetJointDegrees_ & other) const
  {
    if (this->degrees != other.degrees) {
      return false;
    }
    return true;
  }
  bool operator!=(const SetJointDegrees_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SetJointDegrees_

// alias to use template instance with default allocator
using SetJointDegrees =
  joint_positions_node::msg::SetJointDegrees_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace joint_positions_node

#endif  // JOINT_POSITIONS_NODE__MSG__DETAIL__SET_JOINT_DEGREES__STRUCT_HPP_
