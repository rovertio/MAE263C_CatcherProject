// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from dual_dynamixel_node:msg/SetPositions.idl
// generated code does not contain a copyright notice

#ifndef DUAL_DYNAMIXEL_NODE__MSG__DETAIL__SET_POSITIONS__STRUCT_HPP_
#define DUAL_DYNAMIXEL_NODE__MSG__DETAIL__SET_POSITIONS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__dual_dynamixel_node__msg__SetPositions __attribute__((deprecated))
#else
# define DEPRECATED__dual_dynamixel_node__msg__SetPositions __declspec(deprecated)
#endif

namespace dual_dynamixel_node
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct SetPositions_
{
  using Type = SetPositions_<ContainerAllocator>;

  explicit SetPositions_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit SetPositions_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _id_type =
    std::vector<uint8_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint8_t>>;
  _id_type id;
  using _position_type =
    std::vector<int32_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int32_t>>;
  _position_type position;

  // setters for named parameter idiom
  Type & set__id(
    const std::vector<uint8_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint8_t>> & _arg)
  {
    this->id = _arg;
    return *this;
  }
  Type & set__position(
    const std::vector<int32_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int32_t>> & _arg)
  {
    this->position = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    dual_dynamixel_node::msg::SetPositions_<ContainerAllocator> *;
  using ConstRawPtr =
    const dual_dynamixel_node::msg::SetPositions_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<dual_dynamixel_node::msg::SetPositions_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<dual_dynamixel_node::msg::SetPositions_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      dual_dynamixel_node::msg::SetPositions_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<dual_dynamixel_node::msg::SetPositions_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      dual_dynamixel_node::msg::SetPositions_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<dual_dynamixel_node::msg::SetPositions_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<dual_dynamixel_node::msg::SetPositions_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<dual_dynamixel_node::msg::SetPositions_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__dual_dynamixel_node__msg__SetPositions
    std::shared_ptr<dual_dynamixel_node::msg::SetPositions_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__dual_dynamixel_node__msg__SetPositions
    std::shared_ptr<dual_dynamixel_node::msg::SetPositions_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SetPositions_ & other) const
  {
    if (this->id != other.id) {
      return false;
    }
    if (this->position != other.position) {
      return false;
    }
    return true;
  }
  bool operator!=(const SetPositions_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SetPositions_

// alias to use template instance with default allocator
using SetPositions =
  dual_dynamixel_node::msg::SetPositions_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace dual_dynamixel_node

#endif  // DUAL_DYNAMIXEL_NODE__MSG__DETAIL__SET_POSITIONS__STRUCT_HPP_
