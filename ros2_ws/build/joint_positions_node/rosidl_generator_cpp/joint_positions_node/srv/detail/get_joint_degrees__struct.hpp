// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from joint_positions_node:srv/GetJointDegrees.idl
// generated code does not contain a copyright notice

#ifndef JOINT_POSITIONS_NODE__SRV__DETAIL__GET_JOINT_DEGREES__STRUCT_HPP_
#define JOINT_POSITIONS_NODE__SRV__DETAIL__GET_JOINT_DEGREES__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__joint_positions_node__srv__GetJointDegrees_Request __attribute__((deprecated))
#else
# define DEPRECATED__joint_positions_node__srv__GetJointDegrees_Request __declspec(deprecated)
#endif

namespace joint_positions_node
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct GetJointDegrees_Request_
{
  using Type = GetJointDegrees_Request_<ContainerAllocator>;

  explicit GetJointDegrees_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->dummy = false;
    }
  }

  explicit GetJointDegrees_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->dummy = false;
    }
  }

  // field types and members
  using _dummy_type =
    bool;
  _dummy_type dummy;

  // setters for named parameter idiom
  Type & set__dummy(
    const bool & _arg)
  {
    this->dummy = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    joint_positions_node::srv::GetJointDegrees_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const joint_positions_node::srv::GetJointDegrees_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<joint_positions_node::srv::GetJointDegrees_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<joint_positions_node::srv::GetJointDegrees_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      joint_positions_node::srv::GetJointDegrees_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<joint_positions_node::srv::GetJointDegrees_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      joint_positions_node::srv::GetJointDegrees_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<joint_positions_node::srv::GetJointDegrees_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<joint_positions_node::srv::GetJointDegrees_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<joint_positions_node::srv::GetJointDegrees_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__joint_positions_node__srv__GetJointDegrees_Request
    std::shared_ptr<joint_positions_node::srv::GetJointDegrees_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__joint_positions_node__srv__GetJointDegrees_Request
    std::shared_ptr<joint_positions_node::srv::GetJointDegrees_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GetJointDegrees_Request_ & other) const
  {
    if (this->dummy != other.dummy) {
      return false;
    }
    return true;
  }
  bool operator!=(const GetJointDegrees_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GetJointDegrees_Request_

// alias to use template instance with default allocator
using GetJointDegrees_Request =
  joint_positions_node::srv::GetJointDegrees_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace joint_positions_node


#ifndef _WIN32
# define DEPRECATED__joint_positions_node__srv__GetJointDegrees_Response __attribute__((deprecated))
#else
# define DEPRECATED__joint_positions_node__srv__GetJointDegrees_Response __declspec(deprecated)
#endif

namespace joint_positions_node
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct GetJointDegrees_Response_
{
  using Type = GetJointDegrees_Response_<ContainerAllocator>;

  explicit GetJointDegrees_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      std::fill<typename std::array<double, 2>::iterator, double>(this->degrees.begin(), this->degrees.end(), 0.0);
    }
  }

  explicit GetJointDegrees_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
    joint_positions_node::srv::GetJointDegrees_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const joint_positions_node::srv::GetJointDegrees_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<joint_positions_node::srv::GetJointDegrees_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<joint_positions_node::srv::GetJointDegrees_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      joint_positions_node::srv::GetJointDegrees_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<joint_positions_node::srv::GetJointDegrees_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      joint_positions_node::srv::GetJointDegrees_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<joint_positions_node::srv::GetJointDegrees_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<joint_positions_node::srv::GetJointDegrees_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<joint_positions_node::srv::GetJointDegrees_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__joint_positions_node__srv__GetJointDegrees_Response
    std::shared_ptr<joint_positions_node::srv::GetJointDegrees_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__joint_positions_node__srv__GetJointDegrees_Response
    std::shared_ptr<joint_positions_node::srv::GetJointDegrees_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GetJointDegrees_Response_ & other) const
  {
    if (this->degrees != other.degrees) {
      return false;
    }
    return true;
  }
  bool operator!=(const GetJointDegrees_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GetJointDegrees_Response_

// alias to use template instance with default allocator
using GetJointDegrees_Response =
  joint_positions_node::srv::GetJointDegrees_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace joint_positions_node

namespace joint_positions_node
{

namespace srv
{

struct GetJointDegrees
{
  using Request = joint_positions_node::srv::GetJointDegrees_Request;
  using Response = joint_positions_node::srv::GetJointDegrees_Response;
};

}  // namespace srv

}  // namespace joint_positions_node

#endif  // JOINT_POSITIONS_NODE__SRV__DETAIL__GET_JOINT_DEGREES__STRUCT_HPP_
