// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from leo_rover_custom_interface_package:msg/Gripper.idl
// generated code does not contain a copyright notice
#include "leo_rover_custom_interface_package/msg/detail/gripper__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
leo_rover_custom_interface_package__msg__Gripper__init(leo_rover_custom_interface_package__msg__Gripper * msg)
{
  if (!msg) {
    return false;
  }
  // grip
  return true;
}

void
leo_rover_custom_interface_package__msg__Gripper__fini(leo_rover_custom_interface_package__msg__Gripper * msg)
{
  if (!msg) {
    return;
  }
  // grip
}

bool
leo_rover_custom_interface_package__msg__Gripper__are_equal(const leo_rover_custom_interface_package__msg__Gripper * lhs, const leo_rover_custom_interface_package__msg__Gripper * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // grip
  if (lhs->grip != rhs->grip) {
    return false;
  }
  return true;
}

bool
leo_rover_custom_interface_package__msg__Gripper__copy(
  const leo_rover_custom_interface_package__msg__Gripper * input,
  leo_rover_custom_interface_package__msg__Gripper * output)
{
  if (!input || !output) {
    return false;
  }
  // grip
  output->grip = input->grip;
  return true;
}

leo_rover_custom_interface_package__msg__Gripper *
leo_rover_custom_interface_package__msg__Gripper__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  leo_rover_custom_interface_package__msg__Gripper * msg = (leo_rover_custom_interface_package__msg__Gripper *)allocator.allocate(sizeof(leo_rover_custom_interface_package__msg__Gripper), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(leo_rover_custom_interface_package__msg__Gripper));
  bool success = leo_rover_custom_interface_package__msg__Gripper__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
leo_rover_custom_interface_package__msg__Gripper__destroy(leo_rover_custom_interface_package__msg__Gripper * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    leo_rover_custom_interface_package__msg__Gripper__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
leo_rover_custom_interface_package__msg__Gripper__Sequence__init(leo_rover_custom_interface_package__msg__Gripper__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  leo_rover_custom_interface_package__msg__Gripper * data = NULL;

  if (size) {
    data = (leo_rover_custom_interface_package__msg__Gripper *)allocator.zero_allocate(size, sizeof(leo_rover_custom_interface_package__msg__Gripper), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = leo_rover_custom_interface_package__msg__Gripper__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        leo_rover_custom_interface_package__msg__Gripper__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
leo_rover_custom_interface_package__msg__Gripper__Sequence__fini(leo_rover_custom_interface_package__msg__Gripper__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      leo_rover_custom_interface_package__msg__Gripper__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

leo_rover_custom_interface_package__msg__Gripper__Sequence *
leo_rover_custom_interface_package__msg__Gripper__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  leo_rover_custom_interface_package__msg__Gripper__Sequence * array = (leo_rover_custom_interface_package__msg__Gripper__Sequence *)allocator.allocate(sizeof(leo_rover_custom_interface_package__msg__Gripper__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = leo_rover_custom_interface_package__msg__Gripper__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
leo_rover_custom_interface_package__msg__Gripper__Sequence__destroy(leo_rover_custom_interface_package__msg__Gripper__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    leo_rover_custom_interface_package__msg__Gripper__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
leo_rover_custom_interface_package__msg__Gripper__Sequence__are_equal(const leo_rover_custom_interface_package__msg__Gripper__Sequence * lhs, const leo_rover_custom_interface_package__msg__Gripper__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!leo_rover_custom_interface_package__msg__Gripper__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
leo_rover_custom_interface_package__msg__Gripper__Sequence__copy(
  const leo_rover_custom_interface_package__msg__Gripper__Sequence * input,
  leo_rover_custom_interface_package__msg__Gripper__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(leo_rover_custom_interface_package__msg__Gripper);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    leo_rover_custom_interface_package__msg__Gripper * data =
      (leo_rover_custom_interface_package__msg__Gripper *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!leo_rover_custom_interface_package__msg__Gripper__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          leo_rover_custom_interface_package__msg__Gripper__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!leo_rover_custom_interface_package__msg__Gripper__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
