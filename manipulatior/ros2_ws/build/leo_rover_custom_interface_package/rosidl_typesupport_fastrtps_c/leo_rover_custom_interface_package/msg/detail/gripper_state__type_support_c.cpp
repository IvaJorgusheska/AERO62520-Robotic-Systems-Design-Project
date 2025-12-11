// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from leo_rover_custom_interface_package:msg/GripperState.idl
// generated code does not contain a copyright notice
#include "leo_rover_custom_interface_package/msg/detail/gripper_state__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <cstddef>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/serialization_helpers.hpp"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "leo_rover_custom_interface_package/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "leo_rover_custom_interface_package/msg/detail/gripper_state__struct.h"
#include "leo_rover_custom_interface_package/msg/detail/gripper_state__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif


// forward declare type support functions


using _GripperState__ros_msg_type = leo_rover_custom_interface_package__msg__GripperState;


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_leo_rover_custom_interface_package
bool cdr_serialize_leo_rover_custom_interface_package__msg__GripperState(
  const leo_rover_custom_interface_package__msg__GripperState * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: grip
  {
    cdr << (ros_message->grip ? true : false);
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_leo_rover_custom_interface_package
bool cdr_deserialize_leo_rover_custom_interface_package__msg__GripperState(
  eprosima::fastcdr::Cdr & cdr,
  leo_rover_custom_interface_package__msg__GripperState * ros_message)
{
  // Field name: grip
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->grip = tmp ? true : false;
  }

  return true;
}  // NOLINT(readability/fn_size)


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_leo_rover_custom_interface_package
size_t get_serialized_size_leo_rover_custom_interface_package__msg__GripperState(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _GripperState__ros_msg_type * ros_message = static_cast<const _GripperState__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: grip
  {
    size_t item_size = sizeof(ros_message->grip);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_leo_rover_custom_interface_package
size_t max_serialized_size_leo_rover_custom_interface_package__msg__GripperState(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // Field name: grip
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }


  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = leo_rover_custom_interface_package__msg__GripperState;
    is_plain =
      (
      offsetof(DataType, grip) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_leo_rover_custom_interface_package
bool cdr_serialize_key_leo_rover_custom_interface_package__msg__GripperState(
  const leo_rover_custom_interface_package__msg__GripperState * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: grip
  {
    cdr << (ros_message->grip ? true : false);
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_leo_rover_custom_interface_package
size_t get_serialized_size_key_leo_rover_custom_interface_package__msg__GripperState(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _GripperState__ros_msg_type * ros_message = static_cast<const _GripperState__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;

  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: grip
  {
    size_t item_size = sizeof(ros_message->grip);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_leo_rover_custom_interface_package
size_t max_serialized_size_key_leo_rover_custom_interface_package__msg__GripperState(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;
  // Field name: grip
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = leo_rover_custom_interface_package__msg__GripperState;
    is_plain =
      (
      offsetof(DataType, grip) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}


static bool _GripperState__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const leo_rover_custom_interface_package__msg__GripperState * ros_message = static_cast<const leo_rover_custom_interface_package__msg__GripperState *>(untyped_ros_message);
  (void)ros_message;
  return cdr_serialize_leo_rover_custom_interface_package__msg__GripperState(ros_message, cdr);
}

static bool _GripperState__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  leo_rover_custom_interface_package__msg__GripperState * ros_message = static_cast<leo_rover_custom_interface_package__msg__GripperState *>(untyped_ros_message);
  (void)ros_message;
  return cdr_deserialize_leo_rover_custom_interface_package__msg__GripperState(cdr, ros_message);
}

static uint32_t _GripperState__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_leo_rover_custom_interface_package__msg__GripperState(
      untyped_ros_message, 0));
}

static size_t _GripperState__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_leo_rover_custom_interface_package__msg__GripperState(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_GripperState = {
  "leo_rover_custom_interface_package::msg",
  "GripperState",
  _GripperState__cdr_serialize,
  _GripperState__cdr_deserialize,
  _GripperState__get_serialized_size,
  _GripperState__max_serialized_size,
  nullptr
};

static rosidl_message_type_support_t _GripperState__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_GripperState,
  get_message_typesupport_handle_function,
  &leo_rover_custom_interface_package__msg__GripperState__get_type_hash,
  &leo_rover_custom_interface_package__msg__GripperState__get_type_description,
  &leo_rover_custom_interface_package__msg__GripperState__get_type_description_sources,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, leo_rover_custom_interface_package, msg, GripperState)() {
  return &_GripperState__type_support;
}

#if defined(__cplusplus)
}
#endif
