// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from leo_rover_custom_interface_package:msg/GripperState.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "leo_rover_custom_interface_package/msg/gripper_state.hpp"


#ifndef LEO_ROVER_CUSTOM_INTERFACE_PACKAGE__MSG__DETAIL__GRIPPER_STATE__STRUCT_HPP_
#define LEO_ROVER_CUSTOM_INTERFACE_PACKAGE__MSG__DETAIL__GRIPPER_STATE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__leo_rover_custom_interface_package__msg__GripperState __attribute__((deprecated))
#else
# define DEPRECATED__leo_rover_custom_interface_package__msg__GripperState __declspec(deprecated)
#endif

namespace leo_rover_custom_interface_package
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct GripperState_
{
  using Type = GripperState_<ContainerAllocator>;

  explicit GripperState_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->grip = false;
    }
  }

  explicit GripperState_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->grip = false;
    }
  }

  // field types and members
  using _grip_type =
    bool;
  _grip_type grip;

  // setters for named parameter idiom
  Type & set__grip(
    const bool & _arg)
  {
    this->grip = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    leo_rover_custom_interface_package::msg::GripperState_<ContainerAllocator> *;
  using ConstRawPtr =
    const leo_rover_custom_interface_package::msg::GripperState_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<leo_rover_custom_interface_package::msg::GripperState_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<leo_rover_custom_interface_package::msg::GripperState_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      leo_rover_custom_interface_package::msg::GripperState_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<leo_rover_custom_interface_package::msg::GripperState_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      leo_rover_custom_interface_package::msg::GripperState_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<leo_rover_custom_interface_package::msg::GripperState_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<leo_rover_custom_interface_package::msg::GripperState_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<leo_rover_custom_interface_package::msg::GripperState_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__leo_rover_custom_interface_package__msg__GripperState
    std::shared_ptr<leo_rover_custom_interface_package::msg::GripperState_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__leo_rover_custom_interface_package__msg__GripperState
    std::shared_ptr<leo_rover_custom_interface_package::msg::GripperState_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GripperState_ & other) const
  {
    if (this->grip != other.grip) {
      return false;
    }
    return true;
  }
  bool operator!=(const GripperState_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GripperState_

// alias to use template instance with default allocator
using GripperState =
  leo_rover_custom_interface_package::msg::GripperState_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace leo_rover_custom_interface_package

#endif  // LEO_ROVER_CUSTOM_INTERFACE_PACKAGE__MSG__DETAIL__GRIPPER_STATE__STRUCT_HPP_
