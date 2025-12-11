// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from leo_rover_custom_interface_package:msg/Gripper.idl
// generated code does not contain a copyright notice
#include "leo_rover_custom_interface_package/msg/detail/gripper__rosidl_typesupport_fastrtps_cpp.hpp"
#include "leo_rover_custom_interface_package/msg/detail/gripper__functions.h"
#include "leo_rover_custom_interface_package/msg/detail/gripper__struct.hpp"

#include <cstddef>
#include <limits>
#include <stdexcept>
#include <string>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/serialization_helpers.hpp"
#include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
#include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions

namespace leo_rover_custom_interface_package
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{


bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_leo_rover_custom_interface_package
cdr_serialize(
  const leo_rover_custom_interface_package::msg::Gripper & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: grip
  cdr << (ros_message.grip ? true : false);

  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_leo_rover_custom_interface_package
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  leo_rover_custom_interface_package::msg::Gripper & ros_message)
{
  // Member: grip
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.grip = tmp ? true : false;
  }

  return true;
}  // NOLINT(readability/fn_size)


size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_leo_rover_custom_interface_package
get_serialized_size(
  const leo_rover_custom_interface_package::msg::Gripper & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: grip
  {
    size_t item_size = sizeof(ros_message.grip);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}


size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_leo_rover_custom_interface_package
max_serialized_size_Gripper(
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

  // Member: grip
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
    using DataType = leo_rover_custom_interface_package::msg::Gripper;
    is_plain =
      (
      offsetof(DataType, grip) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_leo_rover_custom_interface_package
cdr_serialize_key(
  const leo_rover_custom_interface_package::msg::Gripper & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: grip
  cdr << (ros_message.grip ? true : false);

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_leo_rover_custom_interface_package
get_serialized_size_key(
  const leo_rover_custom_interface_package::msg::Gripper & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: grip
  {
    size_t item_size = sizeof(ros_message.grip);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_leo_rover_custom_interface_package
max_serialized_size_key_Gripper(
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

  // Member: grip
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
    using DataType = leo_rover_custom_interface_package::msg::Gripper;
    is_plain =
      (
      offsetof(DataType, grip) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}


static bool _Gripper__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const leo_rover_custom_interface_package::msg::Gripper *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _Gripper__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<leo_rover_custom_interface_package::msg::Gripper *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _Gripper__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const leo_rover_custom_interface_package::msg::Gripper *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _Gripper__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_Gripper(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _Gripper__callbacks = {
  "leo_rover_custom_interface_package::msg",
  "Gripper",
  _Gripper__cdr_serialize,
  _Gripper__cdr_deserialize,
  _Gripper__get_serialized_size,
  _Gripper__max_serialized_size,
  nullptr
};

static rosidl_message_type_support_t _Gripper__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_Gripper__callbacks,
  get_message_typesupport_handle_function,
  &leo_rover_custom_interface_package__msg__Gripper__get_type_hash,
  &leo_rover_custom_interface_package__msg__Gripper__get_type_description,
  &leo_rover_custom_interface_package__msg__Gripper__get_type_description_sources,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace leo_rover_custom_interface_package

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_leo_rover_custom_interface_package
const rosidl_message_type_support_t *
get_message_type_support_handle<leo_rover_custom_interface_package::msg::Gripper>()
{
  return &leo_rover_custom_interface_package::msg::typesupport_fastrtps_cpp::_Gripper__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, leo_rover_custom_interface_package, msg, Gripper)() {
  return &leo_rover_custom_interface_package::msg::typesupport_fastrtps_cpp::_Gripper__handle;
}

#ifdef __cplusplus
}
#endif
