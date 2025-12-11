// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from leo_rover_custom_interface_package:msg/GripperPose.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "leo_rover_custom_interface_package/msg/gripper_pose.h"


#ifndef LEO_ROVER_CUSTOM_INTERFACE_PACKAGE__MSG__DETAIL__GRIPPER_POSE__FUNCTIONS_H_
#define LEO_ROVER_CUSTOM_INTERFACE_PACKAGE__MSG__DETAIL__GRIPPER_POSE__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/action_type_support_struct.h"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_runtime_c/service_type_support_struct.h"
#include "rosidl_runtime_c/type_description/type_description__struct.h"
#include "rosidl_runtime_c/type_description/type_source__struct.h"
#include "rosidl_runtime_c/type_hash.h"
#include "rosidl_runtime_c/visibility_control.h"
#include "leo_rover_custom_interface_package/msg/rosidl_generator_c__visibility_control.h"

#include "leo_rover_custom_interface_package/msg/detail/gripper_pose__struct.h"

/// Initialize msg/GripperPose message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * leo_rover_custom_interface_package__msg__GripperPose
 * )) before or use
 * leo_rover_custom_interface_package__msg__GripperPose__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_leo_rover_custom_interface_package
bool
leo_rover_custom_interface_package__msg__GripperPose__init(leo_rover_custom_interface_package__msg__GripperPose * msg);

/// Finalize msg/GripperPose message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_leo_rover_custom_interface_package
void
leo_rover_custom_interface_package__msg__GripperPose__fini(leo_rover_custom_interface_package__msg__GripperPose * msg);

/// Create msg/GripperPose message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * leo_rover_custom_interface_package__msg__GripperPose__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_leo_rover_custom_interface_package
leo_rover_custom_interface_package__msg__GripperPose *
leo_rover_custom_interface_package__msg__GripperPose__create(void);

/// Destroy msg/GripperPose message.
/**
 * It calls
 * leo_rover_custom_interface_package__msg__GripperPose__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_leo_rover_custom_interface_package
void
leo_rover_custom_interface_package__msg__GripperPose__destroy(leo_rover_custom_interface_package__msg__GripperPose * msg);

/// Check for msg/GripperPose message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_leo_rover_custom_interface_package
bool
leo_rover_custom_interface_package__msg__GripperPose__are_equal(const leo_rover_custom_interface_package__msg__GripperPose * lhs, const leo_rover_custom_interface_package__msg__GripperPose * rhs);

/// Copy a msg/GripperPose message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_leo_rover_custom_interface_package
bool
leo_rover_custom_interface_package__msg__GripperPose__copy(
  const leo_rover_custom_interface_package__msg__GripperPose * input,
  leo_rover_custom_interface_package__msg__GripperPose * output);

/// Retrieve pointer to the hash of the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_leo_rover_custom_interface_package
const rosidl_type_hash_t *
leo_rover_custom_interface_package__msg__GripperPose__get_type_hash(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_leo_rover_custom_interface_package
const rosidl_runtime_c__type_description__TypeDescription *
leo_rover_custom_interface_package__msg__GripperPose__get_type_description(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the single raw source text that defined this type.
ROSIDL_GENERATOR_C_PUBLIC_leo_rover_custom_interface_package
const rosidl_runtime_c__type_description__TypeSource *
leo_rover_custom_interface_package__msg__GripperPose__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the recursive raw sources that defined the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_leo_rover_custom_interface_package
const rosidl_runtime_c__type_description__TypeSource__Sequence *
leo_rover_custom_interface_package__msg__GripperPose__get_type_description_sources(
  const rosidl_message_type_support_t * type_support);

/// Initialize array of msg/GripperPose messages.
/**
 * It allocates the memory for the number of elements and calls
 * leo_rover_custom_interface_package__msg__GripperPose__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_leo_rover_custom_interface_package
bool
leo_rover_custom_interface_package__msg__GripperPose__Sequence__init(leo_rover_custom_interface_package__msg__GripperPose__Sequence * array, size_t size);

/// Finalize array of msg/GripperPose messages.
/**
 * It calls
 * leo_rover_custom_interface_package__msg__GripperPose__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_leo_rover_custom_interface_package
void
leo_rover_custom_interface_package__msg__GripperPose__Sequence__fini(leo_rover_custom_interface_package__msg__GripperPose__Sequence * array);

/// Create array of msg/GripperPose messages.
/**
 * It allocates the memory for the array and calls
 * leo_rover_custom_interface_package__msg__GripperPose__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_leo_rover_custom_interface_package
leo_rover_custom_interface_package__msg__GripperPose__Sequence *
leo_rover_custom_interface_package__msg__GripperPose__Sequence__create(size_t size);

/// Destroy array of msg/GripperPose messages.
/**
 * It calls
 * leo_rover_custom_interface_package__msg__GripperPose__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_leo_rover_custom_interface_package
void
leo_rover_custom_interface_package__msg__GripperPose__Sequence__destroy(leo_rover_custom_interface_package__msg__GripperPose__Sequence * array);

/// Check for msg/GripperPose message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_leo_rover_custom_interface_package
bool
leo_rover_custom_interface_package__msg__GripperPose__Sequence__are_equal(const leo_rover_custom_interface_package__msg__GripperPose__Sequence * lhs, const leo_rover_custom_interface_package__msg__GripperPose__Sequence * rhs);

/// Copy an array of msg/GripperPose messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_leo_rover_custom_interface_package
bool
leo_rover_custom_interface_package__msg__GripperPose__Sequence__copy(
  const leo_rover_custom_interface_package__msg__GripperPose__Sequence * input,
  leo_rover_custom_interface_package__msg__GripperPose__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // LEO_ROVER_CUSTOM_INTERFACE_PACKAGE__MSG__DETAIL__GRIPPER_POSE__FUNCTIONS_H_
