// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from graspnet_ros:srv/UpdateInterestMap.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "graspnet_ros/srv/update_interest_map.hpp"


#ifndef GRASPNET_ROS__SRV__DETAIL__UPDATE_INTEREST_MAP__TRAITS_HPP_
#define GRASPNET_ROS__SRV__DETAIL__UPDATE_INTEREST_MAP__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "graspnet_ros/srv/detail/update_interest_map__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'grasps'
#include "geometry_msgs/msg/detail/pose__traits.hpp"

namespace graspnet_ros
{

namespace srv
{

inline void to_flow_style_yaml(
  const UpdateInterestMap_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: grasps
  {
    if (msg.grasps.size() == 0) {
      out << "grasps: []";
    } else {
      out << "grasps: [";
      size_t pending_items = msg.grasps.size();
      for (auto item : msg.grasps) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: scores
  {
    if (msg.scores.size() == 0) {
      out << "scores: []";
    } else {
      out << "scores: [";
      size_t pending_items = msg.scores.size();
      for (auto item : msg.scores) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const UpdateInterestMap_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: grasps
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.grasps.size() == 0) {
      out << "grasps: []\n";
    } else {
      out << "grasps:\n";
      for (auto item : msg.grasps) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }

  // member: scores
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.scores.size() == 0) {
      out << "scores: []\n";
    } else {
      out << "scores:\n";
      for (auto item : msg.scores) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const UpdateInterestMap_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace graspnet_ros

namespace rosidl_generator_traits
{

[[deprecated("use graspnet_ros::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const graspnet_ros::srv::UpdateInterestMap_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  graspnet_ros::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use graspnet_ros::srv::to_yaml() instead")]]
inline std::string to_yaml(const graspnet_ros::srv::UpdateInterestMap_Request & msg)
{
  return graspnet_ros::srv::to_yaml(msg);
}

template<>
inline const char * data_type<graspnet_ros::srv::UpdateInterestMap_Request>()
{
  return "graspnet_ros::srv::UpdateInterestMap_Request";
}

template<>
inline const char * name<graspnet_ros::srv::UpdateInterestMap_Request>()
{
  return "graspnet_ros/srv/UpdateInterestMap_Request";
}

template<>
struct has_fixed_size<graspnet_ros::srv::UpdateInterestMap_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<graspnet_ros::srv::UpdateInterestMap_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<graspnet_ros::srv::UpdateInterestMap_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace graspnet_ros
{

namespace srv
{

inline void to_flow_style_yaml(
  const UpdateInterestMap_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: success
  {
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const UpdateInterestMap_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: success
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const UpdateInterestMap_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace graspnet_ros

namespace rosidl_generator_traits
{

[[deprecated("use graspnet_ros::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const graspnet_ros::srv::UpdateInterestMap_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  graspnet_ros::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use graspnet_ros::srv::to_yaml() instead")]]
inline std::string to_yaml(const graspnet_ros::srv::UpdateInterestMap_Response & msg)
{
  return graspnet_ros::srv::to_yaml(msg);
}

template<>
inline const char * data_type<graspnet_ros::srv::UpdateInterestMap_Response>()
{
  return "graspnet_ros::srv::UpdateInterestMap_Response";
}

template<>
inline const char * name<graspnet_ros::srv::UpdateInterestMap_Response>()
{
  return "graspnet_ros/srv/UpdateInterestMap_Response";
}

template<>
struct has_fixed_size<graspnet_ros::srv::UpdateInterestMap_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<graspnet_ros::srv::UpdateInterestMap_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<graspnet_ros::srv::UpdateInterestMap_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__traits.hpp"

namespace graspnet_ros
{

namespace srv
{

inline void to_flow_style_yaml(
  const UpdateInterestMap_Event & msg,
  std::ostream & out)
{
  out << "{";
  // member: info
  {
    out << "info: ";
    to_flow_style_yaml(msg.info, out);
    out << ", ";
  }

  // member: request
  {
    if (msg.request.size() == 0) {
      out << "request: []";
    } else {
      out << "request: [";
      size_t pending_items = msg.request.size();
      for (auto item : msg.request) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: response
  {
    if (msg.response.size() == 0) {
      out << "response: []";
    } else {
      out << "response: [";
      size_t pending_items = msg.response.size();
      for (auto item : msg.response) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const UpdateInterestMap_Event & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: info
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "info:\n";
    to_block_style_yaml(msg.info, out, indentation + 2);
  }

  // member: request
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.request.size() == 0) {
      out << "request: []\n";
    } else {
      out << "request:\n";
      for (auto item : msg.request) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }

  // member: response
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.response.size() == 0) {
      out << "response: []\n";
    } else {
      out << "response:\n";
      for (auto item : msg.response) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const UpdateInterestMap_Event & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace graspnet_ros

namespace rosidl_generator_traits
{

[[deprecated("use graspnet_ros::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const graspnet_ros::srv::UpdateInterestMap_Event & msg,
  std::ostream & out, size_t indentation = 0)
{
  graspnet_ros::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use graspnet_ros::srv::to_yaml() instead")]]
inline std::string to_yaml(const graspnet_ros::srv::UpdateInterestMap_Event & msg)
{
  return graspnet_ros::srv::to_yaml(msg);
}

template<>
inline const char * data_type<graspnet_ros::srv::UpdateInterestMap_Event>()
{
  return "graspnet_ros::srv::UpdateInterestMap_Event";
}

template<>
inline const char * name<graspnet_ros::srv::UpdateInterestMap_Event>()
{
  return "graspnet_ros/srv/UpdateInterestMap_Event";
}

template<>
struct has_fixed_size<graspnet_ros::srv::UpdateInterestMap_Event>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<graspnet_ros::srv::UpdateInterestMap_Event>
  : std::integral_constant<bool, has_bounded_size<graspnet_ros::srv::UpdateInterestMap_Request>::value && has_bounded_size<graspnet_ros::srv::UpdateInterestMap_Response>::value && has_bounded_size<service_msgs::msg::ServiceEventInfo>::value> {};

template<>
struct is_message<graspnet_ros::srv::UpdateInterestMap_Event>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<graspnet_ros::srv::UpdateInterestMap>()
{
  return "graspnet_ros::srv::UpdateInterestMap";
}

template<>
inline const char * name<graspnet_ros::srv::UpdateInterestMap>()
{
  return "graspnet_ros/srv/UpdateInterestMap";
}

template<>
struct has_fixed_size<graspnet_ros::srv::UpdateInterestMap>
  : std::integral_constant<
    bool,
    has_fixed_size<graspnet_ros::srv::UpdateInterestMap_Request>::value &&
    has_fixed_size<graspnet_ros::srv::UpdateInterestMap_Response>::value
  >
{
};

template<>
struct has_bounded_size<graspnet_ros::srv::UpdateInterestMap>
  : std::integral_constant<
    bool,
    has_bounded_size<graspnet_ros::srv::UpdateInterestMap_Request>::value &&
    has_bounded_size<graspnet_ros::srv::UpdateInterestMap_Response>::value
  >
{
};

template<>
struct is_service<graspnet_ros::srv::UpdateInterestMap>
  : std::true_type
{
};

template<>
struct is_service_request<graspnet_ros::srv::UpdateInterestMap_Request>
  : std::true_type
{
};

template<>
struct is_service_response<graspnet_ros::srv::UpdateInterestMap_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // GRASPNET_ROS__SRV__DETAIL__UPDATE_INTEREST_MAP__TRAITS_HPP_
