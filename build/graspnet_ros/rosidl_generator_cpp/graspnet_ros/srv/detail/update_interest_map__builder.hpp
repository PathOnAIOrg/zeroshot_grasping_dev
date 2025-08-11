// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from graspnet_ros:srv/UpdateInterestMap.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "graspnet_ros/srv/update_interest_map.hpp"


#ifndef GRASPNET_ROS__SRV__DETAIL__UPDATE_INTEREST_MAP__BUILDER_HPP_
#define GRASPNET_ROS__SRV__DETAIL__UPDATE_INTEREST_MAP__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "graspnet_ros/srv/detail/update_interest_map__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace graspnet_ros
{

namespace srv
{

namespace builder
{

class Init_UpdateInterestMap_Request_scores
{
public:
  explicit Init_UpdateInterestMap_Request_scores(::graspnet_ros::srv::UpdateInterestMap_Request & msg)
  : msg_(msg)
  {}
  ::graspnet_ros::srv::UpdateInterestMap_Request scores(::graspnet_ros::srv::UpdateInterestMap_Request::_scores_type arg)
  {
    msg_.scores = std::move(arg);
    return std::move(msg_);
  }

private:
  ::graspnet_ros::srv::UpdateInterestMap_Request msg_;
};

class Init_UpdateInterestMap_Request_grasps
{
public:
  Init_UpdateInterestMap_Request_grasps()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_UpdateInterestMap_Request_scores grasps(::graspnet_ros::srv::UpdateInterestMap_Request::_grasps_type arg)
  {
    msg_.grasps = std::move(arg);
    return Init_UpdateInterestMap_Request_scores(msg_);
  }

private:
  ::graspnet_ros::srv::UpdateInterestMap_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::graspnet_ros::srv::UpdateInterestMap_Request>()
{
  return graspnet_ros::srv::builder::Init_UpdateInterestMap_Request_grasps();
}

}  // namespace graspnet_ros


namespace graspnet_ros
{

namespace srv
{

namespace builder
{

class Init_UpdateInterestMap_Response_success
{
public:
  Init_UpdateInterestMap_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::graspnet_ros::srv::UpdateInterestMap_Response success(::graspnet_ros::srv::UpdateInterestMap_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return std::move(msg_);
  }

private:
  ::graspnet_ros::srv::UpdateInterestMap_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::graspnet_ros::srv::UpdateInterestMap_Response>()
{
  return graspnet_ros::srv::builder::Init_UpdateInterestMap_Response_success();
}

}  // namespace graspnet_ros


namespace graspnet_ros
{

namespace srv
{

namespace builder
{

class Init_UpdateInterestMap_Event_response
{
public:
  explicit Init_UpdateInterestMap_Event_response(::graspnet_ros::srv::UpdateInterestMap_Event & msg)
  : msg_(msg)
  {}
  ::graspnet_ros::srv::UpdateInterestMap_Event response(::graspnet_ros::srv::UpdateInterestMap_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::graspnet_ros::srv::UpdateInterestMap_Event msg_;
};

class Init_UpdateInterestMap_Event_request
{
public:
  explicit Init_UpdateInterestMap_Event_request(::graspnet_ros::srv::UpdateInterestMap_Event & msg)
  : msg_(msg)
  {}
  Init_UpdateInterestMap_Event_response request(::graspnet_ros::srv::UpdateInterestMap_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_UpdateInterestMap_Event_response(msg_);
  }

private:
  ::graspnet_ros::srv::UpdateInterestMap_Event msg_;
};

class Init_UpdateInterestMap_Event_info
{
public:
  Init_UpdateInterestMap_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_UpdateInterestMap_Event_request info(::graspnet_ros::srv::UpdateInterestMap_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_UpdateInterestMap_Event_request(msg_);
  }

private:
  ::graspnet_ros::srv::UpdateInterestMap_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::graspnet_ros::srv::UpdateInterestMap_Event>()
{
  return graspnet_ros::srv::builder::Init_UpdateInterestMap_Event_info();
}

}  // namespace graspnet_ros

#endif  // GRASPNET_ROS__SRV__DETAIL__UPDATE_INTEREST_MAP__BUILDER_HPP_
