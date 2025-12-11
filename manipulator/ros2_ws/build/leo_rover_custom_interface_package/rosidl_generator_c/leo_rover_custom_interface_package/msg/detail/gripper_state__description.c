// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from leo_rover_custom_interface_package:msg/GripperState.idl
// generated code does not contain a copyright notice

#include "leo_rover_custom_interface_package/msg/detail/gripper_state__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_leo_rover_custom_interface_package
const rosidl_type_hash_t *
leo_rover_custom_interface_package__msg__GripperState__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x8b, 0x82, 0xaf, 0xd7, 0x6e, 0xcc, 0x05, 0x59,
      0xc3, 0xbe, 0x67, 0xad, 0xaa, 0x4f, 0xc6, 0x2c,
      0x6b, 0xd1, 0x83, 0xd2, 0xeb, 0x54, 0x95, 0x7a,
      0x4d, 0x77, 0x04, 0x21, 0x61, 0x0d, 0x4a, 0x97,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char leo_rover_custom_interface_package__msg__GripperState__TYPE_NAME[] = "leo_rover_custom_interface_package/msg/GripperState";

// Define type names, field names, and default values
static char leo_rover_custom_interface_package__msg__GripperState__FIELD_NAME__grip[] = "grip";

static rosidl_runtime_c__type_description__Field leo_rover_custom_interface_package__msg__GripperState__FIELDS[] = {
  {
    {leo_rover_custom_interface_package__msg__GripperState__FIELD_NAME__grip, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
leo_rover_custom_interface_package__msg__GripperState__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {leo_rover_custom_interface_package__msg__GripperState__TYPE_NAME, 51, 51},
      {leo_rover_custom_interface_package__msg__GripperState__FIELDS, 1, 1},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# GripperState.msg\n"
  "bool grip";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
leo_rover_custom_interface_package__msg__GripperState__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {leo_rover_custom_interface_package__msg__GripperState__TYPE_NAME, 51, 51},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 28, 28},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
leo_rover_custom_interface_package__msg__GripperState__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *leo_rover_custom_interface_package__msg__GripperState__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
