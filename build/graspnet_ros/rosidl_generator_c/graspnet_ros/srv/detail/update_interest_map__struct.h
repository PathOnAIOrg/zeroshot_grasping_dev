// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from graspnet_ros:srv/UpdateInterestMap.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "graspnet_ros/srv/update_interest_map.h"


#ifndef GRASPNET_ROS__SRV__DETAIL__UPDATE_INTEREST_MAP__STRUCT_H_
#define GRASPNET_ROS__SRV__DETAIL__UPDATE_INTEREST_MAP__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'grasps'
#include "geometry_msgs/msg/detail/pose__struct.h"
// Member 'scores'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in srv/UpdateInterestMap in the package graspnet_ros.
typedef struct graspnet_ros__srv__UpdateInterestMap_Request
{
  geometry_msgs__msg__Pose__Sequence grasps;
  rosidl_runtime_c__float__Sequence scores;
} graspnet_ros__srv__UpdateInterestMap_Request;

// Struct for a sequence of graspnet_ros__srv__UpdateInterestMap_Request.
typedef struct graspnet_ros__srv__UpdateInterestMap_Request__Sequence
{
  graspnet_ros__srv__UpdateInterestMap_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} graspnet_ros__srv__UpdateInterestMap_Request__Sequence;

// Constants defined in the message

/// Struct defined in srv/UpdateInterestMap in the package graspnet_ros.
typedef struct graspnet_ros__srv__UpdateInterestMap_Response
{
  bool success;
} graspnet_ros__srv__UpdateInterestMap_Response;

// Struct for a sequence of graspnet_ros__srv__UpdateInterestMap_Response.
typedef struct graspnet_ros__srv__UpdateInterestMap_Response__Sequence
{
  graspnet_ros__srv__UpdateInterestMap_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} graspnet_ros__srv__UpdateInterestMap_Response__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.h"

// constants for array fields with an upper bound
// request
enum
{
  graspnet_ros__srv__UpdateInterestMap_Event__request__MAX_SIZE = 1
};
// response
enum
{
  graspnet_ros__srv__UpdateInterestMap_Event__response__MAX_SIZE = 1
};

/// Struct defined in srv/UpdateInterestMap in the package graspnet_ros.
typedef struct graspnet_ros__srv__UpdateInterestMap_Event
{
  service_msgs__msg__ServiceEventInfo info;
  graspnet_ros__srv__UpdateInterestMap_Request__Sequence request;
  graspnet_ros__srv__UpdateInterestMap_Response__Sequence response;
} graspnet_ros__srv__UpdateInterestMap_Event;

// Struct for a sequence of graspnet_ros__srv__UpdateInterestMap_Event.
typedef struct graspnet_ros__srv__UpdateInterestMap_Event__Sequence
{
  graspnet_ros__srv__UpdateInterestMap_Event * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} graspnet_ros__srv__UpdateInterestMap_Event__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // GRASPNET_ROS__SRV__DETAIL__UPDATE_INTEREST_MAP__STRUCT_H_
