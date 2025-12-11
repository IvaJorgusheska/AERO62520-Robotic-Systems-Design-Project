// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from leo_rover_custom_interface_package:msg/GripperPose.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "leo_rover_custom_interface_package/msg/gripper_pose.hpp"


#ifndef LEO_ROVER_CUSTOM_INTERFACE_PACKAGE__MSG__DETAIL__GRIPPER_POSE__BUILDER_HPP_
#define LEO_ROVER_CUSTOM_INTERFACE_PACKAGE__MSG__DETAIL__GRIPPER_POSE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "leo_rover_custom_interface_package/msg/detail/gripper_pose__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace leo_rover_custom_interface_package
{

namespace msg
{

namespace builder
{

class Init_GripperPose_yaw
{
public:
  explicit Init_GripperPose_yaw(::leo_rover_custom_interface_package::msg::GripperPose & msg)
  : msg_(msg)
  {}
  ::leo_rover_custom_interface_package::msg::GripperPose yaw(::leo_rover_custom_interface_package::msg::GripperPose::_yaw_type arg)
  {
    msg_.yaw = std::move(arg);
    return std::move(msg_);
  }

private:
  ::leo_rover_custom_interface_package::msg::GripperPose msg_;
};

class Init_GripperPose_pitch
{
public:
  explicit Init_GripperPose_pitch(::leo_rover_custom_interface_package::msg::GripperPose & msg)
  : msg_(msg)
  {}
  Init_GripperPose_yaw pitch(::leo_rover_custom_interface_package::msg::GripperPose::_pitch_type arg)
  {
    msg_.pitch = std::move(arg);
    return Init_GripperPose_yaw(msg_);
  }

private:
  ::leo_rover_custom_interface_package::msg::GripperPose msg_;
};

class Init_GripperPose_roll
{
public:
  explicit Init_GripperPose_roll(::leo_rover_custom_interface_package::msg::GripperPose & msg)
  : msg_(msg)
  {}
  Init_GripperPose_pitch roll(::leo_rover_custom_interface_package::msg::GripperPose::_roll_type arg)
  {
    msg_.roll = std::move(arg);
    return Init_GripperPose_pitch(msg_);
  }

private:
  ::leo_rover_custom_interface_package::msg::GripperPose msg_;
};

class Init_GripperPose_z
{
public:
  explicit Init_GripperPose_z(::leo_rover_custom_interface_package::msg::GripperPose & msg)
  : msg_(msg)
  {}
  Init_GripperPose_roll z(::leo_rover_custom_interface_package::msg::GripperPose::_z_type arg)
  {
    msg_.z = std::move(arg);
    return Init_GripperPose_roll(msg_);
  }

private:
  ::leo_rover_custom_interface_package::msg::GripperPose msg_;
};

class Init_GripperPose_y
{
public:
  explicit Init_GripperPose_y(::leo_rover_custom_interface_package::msg::GripperPose & msg)
  : msg_(msg)
  {}
  Init_GripperPose_z y(::leo_rover_custom_interface_package::msg::GripperPose::_y_type arg)
  {
    msg_.y = std::move(arg);
    return Init_GripperPose_z(msg_);
  }

private:
  ::leo_rover_custom_interface_package::msg::GripperPose msg_;
};

class Init_GripperPose_x
{
public:
  Init_GripperPose_x()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GripperPose_y x(::leo_rover_custom_interface_package::msg::GripperPose::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_GripperPose_y(msg_);
  }

private:
  ::leo_rover_custom_interface_package::msg::GripperPose msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::leo_rover_custom_interface_package::msg::GripperPose>()
{
  return leo_rover_custom_interface_package::msg::builder::Init_GripperPose_x();
}

}  // namespace leo_rover_custom_interface_package

#endif  // LEO_ROVER_CUSTOM_INTERFACE_PACKAGE__MSG__DETAIL__GRIPPER_POSE__BUILDER_HPP_
