// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from leo_rover_custom_interface_package:msg/GripperPose.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "leo_rover_custom_interface_package/msg/gripper_pose.hpp"


#ifndef LEO_ROVER_CUSTOM_INTERFACE_PACKAGE__MSG__DETAIL__GRIPPER_POSE__TRAITS_HPP_
#define LEO_ROVER_CUSTOM_INTERFACE_PACKAGE__MSG__DETAIL__GRIPPER_POSE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "leo_rover_custom_interface_package/msg/detail/gripper_pose__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace leo_rover_custom_interface_package
{

namespace msg
{

inline void to_flow_style_yaml(
  const GripperPose & msg,
  std::ostream & out)
{
  out << "{";
  // member: x
  {
    out << "x: ";
    rosidl_generator_traits::value_to_yaml(msg.x, out);
    out << ", ";
  }

  // member: y
  {
    out << "y: ";
    rosidl_generator_traits::value_to_yaml(msg.y, out);
    out << ", ";
  }

  // member: z
  {
    out << "z: ";
    rosidl_generator_traits::value_to_yaml(msg.z, out);
    out << ", ";
  }

  // member: roll
  {
    out << "roll: ";
    rosidl_generator_traits::value_to_yaml(msg.roll, out);
    out << ", ";
  }

  // member: pitch
  {
    out << "pitch: ";
    rosidl_generator_traits::value_to_yaml(msg.pitch, out);
    out << ", ";
  }

  // member: yaw
  {
    out << "yaw: ";
    rosidl_generator_traits::value_to_yaml(msg.yaw, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GripperPose & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "x: ";
    rosidl_generator_traits::value_to_yaml(msg.x, out);
    out << "\n";
  }

  // member: y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "y: ";
    rosidl_generator_traits::value_to_yaml(msg.y, out);
    out << "\n";
  }

  // member: z
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "z: ";
    rosidl_generator_traits::value_to_yaml(msg.z, out);
    out << "\n";
  }

  // member: roll
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "roll: ";
    rosidl_generator_traits::value_to_yaml(msg.roll, out);
    out << "\n";
  }

  // member: pitch
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pitch: ";
    rosidl_generator_traits::value_to_yaml(msg.pitch, out);
    out << "\n";
  }

  // member: yaw
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "yaw: ";
    rosidl_generator_traits::value_to_yaml(msg.yaw, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GripperPose & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace leo_rover_custom_interface_package

namespace rosidl_generator_traits
{

[[deprecated("use leo_rover_custom_interface_package::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const leo_rover_custom_interface_package::msg::GripperPose & msg,
  std::ostream & out, size_t indentation = 0)
{
  leo_rover_custom_interface_package::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use leo_rover_custom_interface_package::msg::to_yaml() instead")]]
inline std::string to_yaml(const leo_rover_custom_interface_package::msg::GripperPose & msg)
{
  return leo_rover_custom_interface_package::msg::to_yaml(msg);
}

template<>
inline const char * data_type<leo_rover_custom_interface_package::msg::GripperPose>()
{
  return "leo_rover_custom_interface_package::msg::GripperPose";
}

template<>
inline const char * name<leo_rover_custom_interface_package::msg::GripperPose>()
{
  return "leo_rover_custom_interface_package/msg/GripperPose";
}

template<>
struct has_fixed_size<leo_rover_custom_interface_package::msg::GripperPose>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<leo_rover_custom_interface_package::msg::GripperPose>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<leo_rover_custom_interface_package::msg::GripperPose>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // LEO_ROVER_CUSTOM_INTERFACE_PACKAGE__MSG__DETAIL__GRIPPER_POSE__TRAITS_HPP_
