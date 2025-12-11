// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from leo_rover_custom_interface_package:msg/Gripper.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "leo_rover_custom_interface_package/msg/gripper.hpp"


#ifndef LEO_ROVER_CUSTOM_INTERFACE_PACKAGE__MSG__DETAIL__GRIPPER__BUILDER_HPP_
#define LEO_ROVER_CUSTOM_INTERFACE_PACKAGE__MSG__DETAIL__GRIPPER__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "leo_rover_custom_interface_package/msg/detail/gripper__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace leo_rover_custom_interface_package
{

namespace msg
{

namespace builder
{

class Init_Gripper_grip
{
public:
  Init_Gripper_grip()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::leo_rover_custom_interface_package::msg::Gripper grip(::leo_rover_custom_interface_package::msg::Gripper::_grip_type arg)
  {
    msg_.grip = std::move(arg);
    return std::move(msg_);
  }

private:
  ::leo_rover_custom_interface_package::msg::Gripper msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::leo_rover_custom_interface_package::msg::Gripper>()
{
  return leo_rover_custom_interface_package::msg::builder::Init_Gripper_grip();
}

}  // namespace leo_rover_custom_interface_package

#endif  // LEO_ROVER_CUSTOM_INTERFACE_PACKAGE__MSG__DETAIL__GRIPPER__BUILDER_HPP_
