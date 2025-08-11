// generated from rosidl_generator_c/resource/idl__type_support.h.em
// with input from graspnet_ros:srv/UpdateInterestMap.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "graspnet_ros/srv/update_interest_map.h"


#ifndef GRASPNET_ROS__SRV__DETAIL__UPDATE_INTEREST_MAP__TYPE_SUPPORT_H_
#define GRASPNET_ROS__SRV__DETAIL__UPDATE_INTEREST_MAP__TYPE_SUPPORT_H_

#include "rosidl_typesupport_interface/macros.h"

#include "graspnet_ros/msg/rosidl_generator_c__visibility_control.h"

#ifdef __cplusplus
extern "C"
{
#endif

#include "rosidl_runtime_c/message_type_support_struct.h"

// Forward declare the get type support functions for this type.
ROSIDL_GENERATOR_C_PUBLIC_graspnet_ros
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
  rosidl_typesupport_c,
  graspnet_ros,
  srv,
  UpdateInterestMap_Request
)(void);

// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"

// Forward declare the get type support functions for this type.
ROSIDL_GENERATOR_C_PUBLIC_graspnet_ros
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
  rosidl_typesupport_c,
  graspnet_ros,
  srv,
  UpdateInterestMap_Response
)(void);

// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"

// Forward declare the get type support functions for this type.
ROSIDL_GENERATOR_C_PUBLIC_graspnet_ros
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
  rosidl_typesupport_c,
  graspnet_ros,
  srv,
  UpdateInterestMap_Event
)(void);

#include "rosidl_runtime_c/service_type_support_struct.h"

// Forward declare the get type support functions for this type.
ROSIDL_GENERATOR_C_PUBLIC_graspnet_ros
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(
  rosidl_typesupport_c,
  graspnet_ros,
  srv,
  UpdateInterestMap
)(void);

// Forward declare the function to create a service event message for this type.
ROSIDL_GENERATOR_C_PUBLIC_graspnet_ros
void *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_CREATE_EVENT_MESSAGE_SYMBOL_NAME(
  rosidl_typesupport_c,
  graspnet_ros,
  srv,
  UpdateInterestMap
)(
  const rosidl_service_introspection_info_t * info,
  rcutils_allocator_t * allocator,
  const void * request_message,
  const void * response_message);

// Forward declare the function to destroy a service event message for this type.
ROSIDL_GENERATOR_C_PUBLIC_graspnet_ros
bool
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_DESTROY_EVENT_MESSAGE_SYMBOL_NAME(
  rosidl_typesupport_c,
  graspnet_ros,
  srv,
  UpdateInterestMap
)(
  void * event_msg,
  rcutils_allocator_t * allocator);

#ifdef __cplusplus
}
#endif

#endif  // GRASPNET_ROS__SRV__DETAIL__UPDATE_INTEREST_MAP__TYPE_SUPPORT_H_
