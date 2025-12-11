// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from leo_rover_custom_interface_package:msg/GripperPose.idl
// generated code does not contain a copyright notice

#include "leo_rover_custom_interface_package/msg/detail/gripper_pose__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_leo_rover_custom_interface_package
const rosidl_type_hash_t *
leo_rover_custom_interface_package__msg__GripperPose__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xab, 0xe4, 0x58, 0xb3, 0xbd, 0xdb, 0x5d, 0xca,
      0x1a, 0x76, 0x3e, 0xd4, 0x53, 0xd0, 0x7d, 0x27,
      0xa8, 0x99, 0x43, 0x93, 0xa0, 0xa4, 0x5d, 0x23,
      0xea, 0xb4, 0xa4, 0x3a, 0x3e, 0x86, 0x19, 0x4f,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char leo_rover_custom_interface_package__msg__GripperPose__TYPE_NAME[] = "leo_rover_custom_interface_package/msg/GripperPose";

// Define type names, field names, and default values
static char leo_rover_custom_interface_package__msg__GripperPose__FIELD_NAME__x[] = "x";
static char leo_rover_custom_interface_package__msg__GripperPose__FIELD_NAME__y[] = "y";
static char leo_rover_custom_interface_package__msg__GripperPose__FIELD_NAME__z[] = "z";
static char leo_rover_custom_interface_package__msg__GripperPose__FIELD_NAME__roll[] = "roll";
static char leo_rover_custom_interface_package__msg__GripperPose__FIELD_NAME__pitch[] = "pitch";
static char leo_rover_custom_interface_package__msg__GripperPose__FIELD_NAME__yaw[] = "yaw";

static rosidl_runtime_c__type_description__Field leo_rover_custom_interface_package__msg__GripperPose__FIELDS[] = {
  {
    {leo_rover_custom_interface_package__msg__GripperPose__FIELD_NAME__x, 1, 1},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {leo_rover_custom_interface_package__msg__GripperPose__FIELD_NAME__y, 1, 1},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {leo_rover_custom_interface_package__msg__GripperPose__FIELD_NAME__z, 1, 1},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {leo_rover_custom_interface_package__msg__GripperPose__FIELD_NAME__roll, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {leo_rover_custom_interface_package__msg__GripperPose__FIELD_NAME__pitch, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {leo_rover_custom_interface_package__msg__GripperPose__FIELD_NAME__yaw, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
leo_rover_custom_interface_package__msg__GripperPose__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {leo_rover_custom_interface_package__msg__GripperPose__TYPE_NAME, 50, 50},
      {leo_rover_custom_interface_package__msg__GripperPose__FIELDS, 6, 6},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# GripperPose.msg\n"
  "float64 x\n"
  "float64 y\n"
  "float64 z\n"
  "float64 roll\n"
  "float64 pitch\n"
  "float64 yaw";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
leo_rover_custom_interface_package__msg__GripperPose__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {leo_rover_custom_interface_package__msg__GripperPose__TYPE_NAME, 50, 50},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 86, 86},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
leo_rover_custom_interface_package__msg__GripperPose__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *leo_rover_custom_interface_package__msg__GripperPose__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
