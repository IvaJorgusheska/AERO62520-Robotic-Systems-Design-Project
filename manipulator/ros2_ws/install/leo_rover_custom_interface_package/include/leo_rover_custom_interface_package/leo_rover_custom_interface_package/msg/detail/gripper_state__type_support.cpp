// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from leo_rover_custom_interface_package:msg/GripperState.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "leo_rover_custom_interface_package/msg/detail/gripper_state__functions.h"
#include "leo_rover_custom_interface_package/msg/detail/gripper_state__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace leo_rover_custom_interface_package
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void GripperState_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) leo_rover_custom_interface_package::msg::GripperState(_init);
}

void GripperState_fini_function(void * message_memory)
{
  auto typed_message = static_cast<leo_rover_custom_interface_package::msg::GripperState *>(message_memory);
  typed_message->~GripperState();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember GripperState_message_member_array[1] = {
  {
    "grip",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(leo_rover_custom_interface_package::msg::GripperState, grip),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers GripperState_message_members = {
  "leo_rover_custom_interface_package::msg",  // message namespace
  "GripperState",  // message name
  1,  // number of fields
  sizeof(leo_rover_custom_interface_package::msg::GripperState),
  false,  // has_any_key_member_
  GripperState_message_member_array,  // message members
  GripperState_init_function,  // function to initialize message memory (memory has to be allocated)
  GripperState_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t GripperState_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &GripperState_message_members,
  get_message_typesupport_handle_function,
  &leo_rover_custom_interface_package__msg__GripperState__get_type_hash,
  &leo_rover_custom_interface_package__msg__GripperState__get_type_description,
  &leo_rover_custom_interface_package__msg__GripperState__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace leo_rover_custom_interface_package


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<leo_rover_custom_interface_package::msg::GripperState>()
{
  return &::leo_rover_custom_interface_package::msg::rosidl_typesupport_introspection_cpp::GripperState_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, leo_rover_custom_interface_package, msg, GripperState)() {
  return &::leo_rover_custom_interface_package::msg::rosidl_typesupport_introspection_cpp::GripperState_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
