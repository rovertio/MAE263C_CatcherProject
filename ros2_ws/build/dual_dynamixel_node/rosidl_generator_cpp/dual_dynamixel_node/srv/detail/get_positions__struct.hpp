// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from dual_dynamixel_node:srv/GetPositions.idl
// generated code does not contain a copyright notice

#ifndef DUAL_DYNAMIXEL_NODE__SRV__DETAIL__GET_POSITIONS__STRUCT_HPP_
#define DUAL_DYNAMIXEL_NODE__SRV__DETAIL__GET_POSITIONS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__dual_dynamixel_node__srv__GetPositions_Request __attribute__((deprecated))
#else
# define DEPRECATED__dual_dynamixel_node__srv__GetPositions_Request __declspec(deprecated)
#endif

namespace dual_dynamixel_node
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct GetPositions_Request_
{
  using Type = GetPositions_Request_<ContainerAllocator>;

  explicit GetPositions_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit GetPositions_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _id_type =
    std::vector<uint8_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint8_t>>;
  _id_type id;

  // setters for named parameter idiom
  Type & set__id(
    const std::vector<uint8_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint8_t>> & _arg)
  {
    this->id = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    dual_dynamixel_node::srv::GetPositions_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const dual_dynamixel_node::srv::GetPositions_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<dual_dynamixel_node::srv::GetPositions_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<dual_dynamixel_node::srv::GetPositions_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      dual_dynamixel_node::srv::GetPositions_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<dual_dynamixel_node::srv::GetPositions_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      dual_dynamixel_node::srv::GetPositions_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<dual_dynamixel_node::srv::GetPositions_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<dual_dynamixel_node::srv::GetPositions_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<dual_dynamixel_node::srv::GetPositions_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__dual_dynamixel_node__srv__GetPositions_Request
    std::shared_ptr<dual_dynamixel_node::srv::GetPositions_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__dual_dynamixel_node__srv__GetPositions_Request
    std::shared_ptr<dual_dynamixel_node::srv::GetPositions_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GetPositions_Request_ & other) const
  {
    if (this->id != other.id) {
      return false;
    }
    return true;
  }
  bool operator!=(const GetPositions_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GetPositions_Request_

// alias to use template instance with default allocator
using GetPositions_Request =
  dual_dynamixel_node::srv::GetPositions_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace dual_dynamixel_node


#ifndef _WIN32
# define DEPRECATED__dual_dynamixel_node__srv__GetPositions_Response __attribute__((deprecated))
#else
# define DEPRECATED__dual_dynamixel_node__srv__GetPositions_Response __declspec(deprecated)
#endif

namespace dual_dynamixel_node
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct GetPositions_Response_
{
  using Type = GetPositions_Response_<ContainerAllocator>;

  explicit GetPositions_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit GetPositions_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _position_type =
    std::vector<int32_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int32_t>>;
  _position_type position;

  // setters for named parameter idiom
  Type & set__position(
    const std::vector<int32_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int32_t>> & _arg)
  {
    this->position = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    dual_dynamixel_node::srv::GetPositions_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const dual_dynamixel_node::srv::GetPositions_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<dual_dynamixel_node::srv::GetPositions_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<dual_dynamixel_node::srv::GetPositions_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      dual_dynamixel_node::srv::GetPositions_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<dual_dynamixel_node::srv::GetPositions_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      dual_dynamixel_node::srv::GetPositions_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<dual_dynamixel_node::srv::GetPositions_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<dual_dynamixel_node::srv::GetPositions_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<dual_dynamixel_node::srv::GetPositions_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__dual_dynamixel_node__srv__GetPositions_Response
    std::shared_ptr<dual_dynamixel_node::srv::GetPositions_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__dual_dynamixel_node__srv__GetPositions_Response
    std::shared_ptr<dual_dynamixel_node::srv::GetPositions_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GetPositions_Response_ & other) const
  {
    if (this->position != other.position) {
      return false;
    }
    return true;
  }
  bool operator!=(const GetPositions_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GetPositions_Response_

// alias to use template instance with default allocator
using GetPositions_Response =
  dual_dynamixel_node::srv::GetPositions_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace dual_dynamixel_node

namespace dual_dynamixel_node
{

namespace srv
{

struct GetPositions
{
  using Request = dual_dynamixel_node::srv::GetPositions_Request;
  using Response = dual_dynamixel_node::srv::GetPositions_Response;
};

}  // namespace srv

}  // namespace dual_dynamixel_node

#endif  // DUAL_DYNAMIXEL_NODE__SRV__DETAIL__GET_POSITIONS__STRUCT_HPP_
