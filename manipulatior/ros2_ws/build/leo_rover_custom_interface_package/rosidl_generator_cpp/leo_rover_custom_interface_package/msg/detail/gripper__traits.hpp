// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from leo_rover_custom_interface_package:msg/Gripper.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "leo_rover_custom_interface_package/msg/gripper.hpp"


#ifndef LEO_ROVER_CUSTOM_INTERFACE_PACKAGE__MSG__DETAIL__GRIPPER__TRAITS_HPP_
#define LEO_ROVER_CUSTOM_INTERFACE_PACKAGE__MSG__DETAIL__GRIPPER__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "leo_rover_custom_interface_package/msg/detail/gripper__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace leo_rover_custom_interface_package
{

namespace msg
{

inline void to_flow_style_yaml(
  const Gripper & msg,
  std::ostream & out)
{
  out << "{";
  // member: grip
  {
    out << "grip: ";
    rosidl_generator_traits::value_to_yaml(msg.grip, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Gripper & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: grip
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "grip: ";
    rosidl_generator_traits::value_to_yaml(msg.grip, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Gripper & msg, bool use_flow_style = false)
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
  const leo_rover_custom_interface_package::msg::Gripper & msg,
  std::ostream & out, size_t indentation = 0)
{
  leo_rover_custom_interface_package::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use leo_rover_custom_interface_package::msg::to_yaml() instead")]]
inline std::string to_yaml(const leo_rover_custom_interface_package::msg::Gripper & msg)
{
  return leo_rover_custom_interface_package::msg::to_yaml(msg);
}

template<>
inline const char * data_type<leo_rover_custom_interface_package::msg::Gripper>()
{
  return "leo_rover_custom_interface_package::msg::Gripper";
}

template<>
inline const char * name<leo_rover_custom_interface_package::msg::Gripper>()
{
  return "leo_rover_custom_interface_package/msg/Gripper";
}

template<>
struct has_fixed_size<leo_rover_custom_interface_package::msg::Gripper>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<leo_rover_custom_interface_package::msg::Gripper>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<leo_rover_custom_interface_package::msg::Gripper>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // LEO_ROVER_CUSTOM_INTERFACE_PACKAGE__MSG__DETAIL__GRIPPER__TRAITS_HPP_
