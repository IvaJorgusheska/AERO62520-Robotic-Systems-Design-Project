// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from leo_rover_custom_interface_package:msg/Gripper.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "leo_rover_custom_interface_package/msg/gripper.h"


#ifndef LEO_ROVER_CUSTOM_INTERFACE_PACKAGE__MSG__DETAIL__GRIPPER__STRUCT_H_
#define LEO_ROVER_CUSTOM_INTERFACE_PACKAGE__MSG__DETAIL__GRIPPER__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

/// Struct defined in msg/Gripper in the package leo_rover_custom_interface_package.
/**
  * Gripper.msg
 */
typedef struct leo_rover_custom_interface_package__msg__Gripper
{
  bool grip;
} leo_rover_custom_interface_package__msg__Gripper;

// Struct for a sequence of leo_rover_custom_interface_package__msg__Gripper.
typedef struct leo_rover_custom_interface_package__msg__Gripper__Sequence
{
  leo_rover_custom_interface_package__msg__Gripper * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} leo_rover_custom_interface_package__msg__Gripper__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // LEO_ROVER_CUSTOM_INTERFACE_PACKAGE__MSG__DETAIL__GRIPPER__STRUCT_H_
