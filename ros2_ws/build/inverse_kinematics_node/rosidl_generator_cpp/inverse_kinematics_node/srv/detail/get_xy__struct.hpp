// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from inverse_kinematics_node:srv/GetXY.idl
// generated code does not contain a copyright notice

#ifndef INVERSE_KINEMATICS_NODE__SRV__DETAIL__GET_XY__STRUCT_HPP_
#define INVERSE_KINEMATICS_NODE__SRV__DETAIL__GET_XY__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__inverse_kinematics_node__srv__GetXY_Request __attribute__((deprecated))
#else
# define DEPRECATED__inverse_kinematics_node__srv__GetXY_Request __declspec(deprecated)
#endif

namespace inverse_kinematics_node
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct GetXY_Request_
{
  using Type = GetXY_Request_<ContainerAllocator>;

  explicit GetXY_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  explicit GetXY_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  // field types and members
  using _structure_needs_at_least_one_member_type =
    uint8_t;
  _structure_needs_at_least_one_member_type structure_needs_at_least_one_member;


  // constant declarations

  // pointer types
  using RawPtr =
    inverse_kinematics_node::srv::GetXY_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const inverse_kinematics_node::srv::GetXY_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<inverse_kinematics_node::srv::GetXY_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<inverse_kinematics_node::srv::GetXY_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      inverse_kinematics_node::srv::GetXY_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<inverse_kinematics_node::srv::GetXY_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      inverse_kinematics_node::srv::GetXY_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<inverse_kinematics_node::srv::GetXY_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<inverse_kinematics_node::srv::GetXY_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<inverse_kinematics_node::srv::GetXY_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__inverse_kinematics_node__srv__GetXY_Request
    std::shared_ptr<inverse_kinematics_node::srv::GetXY_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__inverse_kinematics_node__srv__GetXY_Request
    std::shared_ptr<inverse_kinematics_node::srv::GetXY_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GetXY_Request_ & other) const
  {
    if (this->structure_needs_at_least_one_member != other.structure_needs_at_least_one_member) {
      return false;
    }
    return true;
  }
  bool operator!=(const GetXY_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GetXY_Request_

// alias to use template instance with default allocator
using GetXY_Request =
  inverse_kinematics_node::srv::GetXY_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace inverse_kinematics_node


#ifndef _WIN32
# define DEPRECATED__inverse_kinematics_node__srv__GetXY_Response __attribute__((deprecated))
#else
# define DEPRECATED__inverse_kinematics_node__srv__GetXY_Response __declspec(deprecated)
#endif

namespace inverse_kinematics_node
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct GetXY_Response_
{
  using Type = GetXY_Response_<ContainerAllocator>;

  explicit GetXY_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->x = 0.0;
      this->y = 0.0;
    }
  }

  explicit GetXY_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
    inverse_kinematics_node::srv::GetXY_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const inverse_kinematics_node::srv::GetXY_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<inverse_kinematics_node::srv::GetXY_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<inverse_kinematics_node::srv::GetXY_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      inverse_kinematics_node::srv::GetXY_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<inverse_kinematics_node::srv::GetXY_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      inverse_kinematics_node::srv::GetXY_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<inverse_kinematics_node::srv::GetXY_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<inverse_kinematics_node::srv::GetXY_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<inverse_kinematics_node::srv::GetXY_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__inverse_kinematics_node__srv__GetXY_Response
    std::shared_ptr<inverse_kinematics_node::srv::GetXY_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__inverse_kinematics_node__srv__GetXY_Response
    std::shared_ptr<inverse_kinematics_node::srv::GetXY_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GetXY_Response_ & other) const
  {
    if (this->x != other.x) {
      return false;
    }
    if (this->y != other.y) {
      return false;
    }
    return true;
  }
  bool operator!=(const GetXY_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GetXY_Response_

// alias to use template instance with default allocator
using GetXY_Response =
  inverse_kinematics_node::srv::GetXY_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace inverse_kinematics_node

namespace inverse_kinematics_node
{

namespace srv
{

struct GetXY
{
  using Request = inverse_kinematics_node::srv::GetXY_Request;
  using Response = inverse_kinematics_node::srv::GetXY_Response;
};

}  // namespace srv

}  // namespace inverse_kinematics_node

#endif  // INVERSE_KINEMATICS_NODE__SRV__DETAIL__GET_XY__STRUCT_HPP_
