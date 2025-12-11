// generated from rosidl_typesupport_fastrtps_c/resource/idl__rosidl_typesupport_fastrtps_c.h.em
// with input from leo_rover_custom_interface_package:msg/GripperState.idl
// generated code does not contain a copyright notice
#ifndef LEO_ROVER_CUSTOM_INTERFACE_PACKAGE__MSG__DETAIL__GRIPPER_STATE__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_
#define LEO_ROVER_CUSTOM_INTERFACE_PACKAGE__MSG__DETAIL__GRIPPER_STATE__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_


#include <stddef.h>
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "leo_rover_custom_interface_package/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "leo_rover_custom_interface_package/msg/detail/gripper_state__struct.h"
#include "fastcdr/Cdr.h"

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_leo_rover_custom_interface_package
bool cdr_serialize_leo_rover_custom_interface_package__msg__GripperState(
  const leo_rover_custom_interface_package__msg__GripperState * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_leo_rover_custom_interface_package
bool cdr_deserialize_leo_rover_custom_interface_package__msg__GripperState(
  eprosima::fastcdr::Cdr &,
  leo_rover_custom_interface_package__msg__GripperState * ros_message);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_leo_rover_custom_interface_package
size_t get_serialized_size_leo_rover_custom_interface_package__msg__GripperState(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_leo_rover_custom_interface_package
size_t max_serialized_size_leo_rover_custom_interface_package__msg__GripperState(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_leo_rover_custom_interface_package
bool cdr_serialize_key_leo_rover_custom_interface_package__msg__GripperState(
  const leo_rover_custom_interface_package__msg__GripperState * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_leo_rover_custom_interface_package
size_t get_serialized_size_key_leo_rover_custom_interface_package__msg__GripperState(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_leo_rover_custom_interface_package
size_t max_serialized_size_key_leo_rover_custom_interface_package__msg__GripperState(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_leo_rover_custom_interface_package
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, leo_rover_custom_interface_package, msg, GripperState)();

#ifdef __cplusplus
}
#endif

#endif  // LEO_ROVER_CUSTOM_INTERFACE_PACKAGE__MSG__DETAIL__GRIPPER_STATE__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_
