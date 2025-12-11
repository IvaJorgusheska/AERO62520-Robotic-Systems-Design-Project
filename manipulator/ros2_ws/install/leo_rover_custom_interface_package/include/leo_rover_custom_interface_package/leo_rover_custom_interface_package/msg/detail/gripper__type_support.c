// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from leo_rover_custom_interface_package:msg/Gripper.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "leo_rover_custom_interface_package/msg/detail/gripper__rosidl_typesupport_introspection_c.h"
#include "leo_rover_custom_interface_package/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "leo_rover_custom_interface_package/msg/detail/gripper__functions.h"
#include "leo_rover_custom_interface_package/msg/detail/gripper__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void leo_rover_custom_interface_package__msg__Gripper__rosidl_typesupport_introspection_c__Gripper_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  leo_rover_custom_interface_package__msg__Gripper__init(message_memory);
}

void leo_rover_custom_interface_package__msg__Gripper__rosidl_typesupport_introspection_c__Gripper_fini_function(void * message_memory)
{
  leo_rover_custom_interface_package__msg__Gripper__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember leo_rover_custom_interface_package__msg__Gripper__rosidl_typesupport_introspection_c__Gripper_message_member_array[1] = {
  {
    "grip",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(leo_rover_custom_interface_package__msg__Gripper, grip),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers leo_rover_custom_interface_package__msg__Gripper__rosidl_typesupport_introspection_c__Gripper_message_members = {
  "leo_rover_custom_interface_package__msg",  // message namespace
  "Gripper",  // message name
  1,  // number of fields
  sizeof(leo_rover_custom_interface_package__msg__Gripper),
  false,  // has_any_key_member_
  leo_rover_custom_interface_package__msg__Gripper__rosidl_typesupport_introspection_c__Gripper_message_member_array,  // message members
  leo_rover_custom_interface_package__msg__Gripper__rosidl_typesupport_introspection_c__Gripper_init_function,  // function to initialize message memory (memory has to be allocated)
  leo_rover_custom_interface_package__msg__Gripper__rosidl_typesupport_introspection_c__Gripper_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t leo_rover_custom_interface_package__msg__Gripper__rosidl_typesupport_introspection_c__Gripper_message_type_support_handle = {
  0,
  &leo_rover_custom_interface_package__msg__Gripper__rosidl_typesupport_introspection_c__Gripper_message_members,
  get_message_typesupport_handle_function,
  &leo_rover_custom_interface_package__msg__Gripper__get_type_hash,
  &leo_rover_custom_interface_package__msg__Gripper__get_type_description,
  &leo_rover_custom_interface_package__msg__Gripper__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_leo_rover_custom_interface_package
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, leo_rover_custom_interface_package, msg, Gripper)() {
  if (!leo_rover_custom_interface_package__msg__Gripper__rosidl_typesupport_introspection_c__Gripper_message_type_support_handle.typesupport_identifier) {
    leo_rover_custom_interface_package__msg__Gripper__rosidl_typesupport_introspection_c__Gripper_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &leo_rover_custom_interface_package__msg__Gripper__rosidl_typesupport_introspection_c__Gripper_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
