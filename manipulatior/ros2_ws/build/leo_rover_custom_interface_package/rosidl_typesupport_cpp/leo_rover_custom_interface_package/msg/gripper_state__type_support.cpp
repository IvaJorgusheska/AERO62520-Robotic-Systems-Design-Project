// generated from rosidl_typesupport_cpp/resource/idl__type_support.cpp.em
// with input from leo_rover_custom_interface_package:msg/GripperState.idl
// generated code does not contain a copyright notice

#include "cstddef"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "leo_rover_custom_interface_package/msg/detail/gripper_state__functions.h"
#include "leo_rover_custom_interface_package/msg/detail/gripper_state__struct.hpp"
#include "rosidl_typesupport_cpp/identifier.hpp"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
#include "rosidl_typesupport_cpp/visibility_control.h"
#include "rosidl_typesupport_interface/macros.h"

namespace leo_rover_custom_interface_package
{

namespace msg
{

namespace rosidl_typesupport_cpp
{

typedef struct _GripperState_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _GripperState_type_support_ids_t;

static const _GripperState_type_support_ids_t _GripperState_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _GripperState_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _GripperState_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _GripperState_type_support_symbol_names_t _GripperState_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, leo_rover_custom_interface_package, msg, GripperState)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, leo_rover_custom_interface_package, msg, GripperState)),
  }
};

typedef struct _GripperState_type_support_data_t
{
  void * data[2];
} _GripperState_type_support_data_t;

static _GripperState_type_support_data_t _GripperState_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _GripperState_message_typesupport_map = {
  2,
  "leo_rover_custom_interface_package",
  &_GripperState_message_typesupport_ids.typesupport_identifier[0],
  &_GripperState_message_typesupport_symbol_names.symbol_name[0],
  &_GripperState_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t GripperState_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_GripperState_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
  &leo_rover_custom_interface_package__msg__GripperState__get_type_hash,
  &leo_rover_custom_interface_package__msg__GripperState__get_type_description,
  &leo_rover_custom_interface_package__msg__GripperState__get_type_description_sources,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace msg

}  // namespace leo_rover_custom_interface_package

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<leo_rover_custom_interface_package::msg::GripperState>()
{
  return &::leo_rover_custom_interface_package::msg::rosidl_typesupport_cpp::GripperState_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, leo_rover_custom_interface_package, msg, GripperState)() {
  return get_message_type_support_handle<leo_rover_custom_interface_package::msg::GripperState>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp
