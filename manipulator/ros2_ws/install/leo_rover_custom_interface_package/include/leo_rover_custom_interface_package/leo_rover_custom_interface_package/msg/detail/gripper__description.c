// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from leo_rover_custom_interface_package:msg/Gripper.idl
// generated code does not contain a copyright notice

#include "leo_rover_custom_interface_package/msg/detail/gripper__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_leo_rover_custom_interface_package
const rosidl_type_hash_t *
leo_rover_custom_interface_package__msg__Gripper__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xe4, 0x6c, 0xae, 0x89, 0x83, 0x8d, 0x13, 0x1a,
      0xf9, 0x30, 0x7d, 0xbb, 0xda, 0x03, 0x5b, 0x80,
      0x36, 0x12, 0xc6, 0x58, 0x9f, 0xae, 0x22, 0xf0,
      0xa2, 0x0a, 0xce, 0xa5, 0xfe, 0xf3, 0x2c, 0x99,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char leo_rover_custom_interface_package__msg__Gripper__TYPE_NAME[] = "leo_rover_custom_interface_package/msg/Gripper";

// Define type names, field names, and default values
static char leo_rover_custom_interface_package__msg__Gripper__FIELD_NAME__grip[] = "grip";

static rosidl_runtime_c__type_description__Field leo_rover_custom_interface_package__msg__Gripper__FIELDS[] = {
  {
    {leo_rover_custom_interface_package__msg__Gripper__FIELD_NAME__grip, 4, 4},
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
leo_rover_custom_interface_package__msg__Gripper__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {leo_rover_custom_interface_package__msg__Gripper__TYPE_NAME, 46, 46},
      {leo_rover_custom_interface_package__msg__Gripper__FIELDS, 1, 1},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# Gripper.msg\n"
  "bool grip";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
leo_rover_custom_interface_package__msg__Gripper__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {leo_rover_custom_interface_package__msg__Gripper__TYPE_NAME, 46, 46},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 23, 23},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
leo_rover_custom_interface_package__msg__Gripper__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *leo_rover_custom_interface_package__msg__Gripper__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
