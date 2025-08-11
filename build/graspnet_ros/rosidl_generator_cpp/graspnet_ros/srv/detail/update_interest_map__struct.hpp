// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from graspnet_ros:srv/UpdateInterestMap.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "graspnet_ros/srv/update_interest_map.hpp"


#ifndef GRASPNET_ROS__SRV__DETAIL__UPDATE_INTEREST_MAP__STRUCT_HPP_
#define GRASPNET_ROS__SRV__DETAIL__UPDATE_INTEREST_MAP__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'grasps'
#include "geometry_msgs/msg/detail/pose__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__graspnet_ros__srv__UpdateInterestMap_Request __attribute__((deprecated))
#else
# define DEPRECATED__graspnet_ros__srv__UpdateInterestMap_Request __declspec(deprecated)
#endif

namespace graspnet_ros
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct UpdateInterestMap_Request_
{
  using Type = UpdateInterestMap_Request_<ContainerAllocator>;

  explicit UpdateInterestMap_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit UpdateInterestMap_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _grasps_type =
    std::vector<geometry_msgs::msg::Pose_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<geometry_msgs::msg::Pose_<ContainerAllocator>>>;
  _grasps_type grasps;
  using _scores_type =
    std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>>;
  _scores_type scores;

  // setters for named parameter idiom
  Type & set__grasps(
    const std::vector<geometry_msgs::msg::Pose_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<geometry_msgs::msg::Pose_<ContainerAllocator>>> & _arg)
  {
    this->grasps = _arg;
    return *this;
  }
  Type & set__scores(
    const std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>> & _arg)
  {
    this->scores = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    graspnet_ros::srv::UpdateInterestMap_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const graspnet_ros::srv::UpdateInterestMap_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<graspnet_ros::srv::UpdateInterestMap_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<graspnet_ros::srv::UpdateInterestMap_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      graspnet_ros::srv::UpdateInterestMap_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<graspnet_ros::srv::UpdateInterestMap_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      graspnet_ros::srv::UpdateInterestMap_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<graspnet_ros::srv::UpdateInterestMap_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<graspnet_ros::srv::UpdateInterestMap_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<graspnet_ros::srv::UpdateInterestMap_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__graspnet_ros__srv__UpdateInterestMap_Request
    std::shared_ptr<graspnet_ros::srv::UpdateInterestMap_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__graspnet_ros__srv__UpdateInterestMap_Request
    std::shared_ptr<graspnet_ros::srv::UpdateInterestMap_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const UpdateInterestMap_Request_ & other) const
  {
    if (this->grasps != other.grasps) {
      return false;
    }
    if (this->scores != other.scores) {
      return false;
    }
    return true;
  }
  bool operator!=(const UpdateInterestMap_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct UpdateInterestMap_Request_

// alias to use template instance with default allocator
using UpdateInterestMap_Request =
  graspnet_ros::srv::UpdateInterestMap_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace graspnet_ros


#ifndef _WIN32
# define DEPRECATED__graspnet_ros__srv__UpdateInterestMap_Response __attribute__((deprecated))
#else
# define DEPRECATED__graspnet_ros__srv__UpdateInterestMap_Response __declspec(deprecated)
#endif

namespace graspnet_ros
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct UpdateInterestMap_Response_
{
  using Type = UpdateInterestMap_Response_<ContainerAllocator>;

  explicit UpdateInterestMap_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
    }
  }

  explicit UpdateInterestMap_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;

  // setters for named parameter idiom
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    graspnet_ros::srv::UpdateInterestMap_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const graspnet_ros::srv::UpdateInterestMap_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<graspnet_ros::srv::UpdateInterestMap_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<graspnet_ros::srv::UpdateInterestMap_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      graspnet_ros::srv::UpdateInterestMap_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<graspnet_ros::srv::UpdateInterestMap_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      graspnet_ros::srv::UpdateInterestMap_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<graspnet_ros::srv::UpdateInterestMap_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<graspnet_ros::srv::UpdateInterestMap_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<graspnet_ros::srv::UpdateInterestMap_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__graspnet_ros__srv__UpdateInterestMap_Response
    std::shared_ptr<graspnet_ros::srv::UpdateInterestMap_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__graspnet_ros__srv__UpdateInterestMap_Response
    std::shared_ptr<graspnet_ros::srv::UpdateInterestMap_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const UpdateInterestMap_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    return true;
  }
  bool operator!=(const UpdateInterestMap_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct UpdateInterestMap_Response_

// alias to use template instance with default allocator
using UpdateInterestMap_Response =
  graspnet_ros::srv::UpdateInterestMap_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace graspnet_ros


// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__graspnet_ros__srv__UpdateInterestMap_Event __attribute__((deprecated))
#else
# define DEPRECATED__graspnet_ros__srv__UpdateInterestMap_Event __declspec(deprecated)
#endif

namespace graspnet_ros
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct UpdateInterestMap_Event_
{
  using Type = UpdateInterestMap_Event_<ContainerAllocator>;

  explicit UpdateInterestMap_Event_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_init)
  {
    (void)_init;
  }

  explicit UpdateInterestMap_Event_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _info_type =
    service_msgs::msg::ServiceEventInfo_<ContainerAllocator>;
  _info_type info;
  using _request_type =
    rosidl_runtime_cpp::BoundedVector<graspnet_ros::srv::UpdateInterestMap_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<graspnet_ros::srv::UpdateInterestMap_Request_<ContainerAllocator>>>;
  _request_type request;
  using _response_type =
    rosidl_runtime_cpp::BoundedVector<graspnet_ros::srv::UpdateInterestMap_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<graspnet_ros::srv::UpdateInterestMap_Response_<ContainerAllocator>>>;
  _response_type response;

  // setters for named parameter idiom
  Type & set__info(
    const service_msgs::msg::ServiceEventInfo_<ContainerAllocator> & _arg)
  {
    this->info = _arg;
    return *this;
  }
  Type & set__request(
    const rosidl_runtime_cpp::BoundedVector<graspnet_ros::srv::UpdateInterestMap_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<graspnet_ros::srv::UpdateInterestMap_Request_<ContainerAllocator>>> & _arg)
  {
    this->request = _arg;
    return *this;
  }
  Type & set__response(
    const rosidl_runtime_cpp::BoundedVector<graspnet_ros::srv::UpdateInterestMap_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<graspnet_ros::srv::UpdateInterestMap_Response_<ContainerAllocator>>> & _arg)
  {
    this->response = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    graspnet_ros::srv::UpdateInterestMap_Event_<ContainerAllocator> *;
  using ConstRawPtr =
    const graspnet_ros::srv::UpdateInterestMap_Event_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<graspnet_ros::srv::UpdateInterestMap_Event_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<graspnet_ros::srv::UpdateInterestMap_Event_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      graspnet_ros::srv::UpdateInterestMap_Event_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<graspnet_ros::srv::UpdateInterestMap_Event_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      graspnet_ros::srv::UpdateInterestMap_Event_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<graspnet_ros::srv::UpdateInterestMap_Event_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<graspnet_ros::srv::UpdateInterestMap_Event_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<graspnet_ros::srv::UpdateInterestMap_Event_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__graspnet_ros__srv__UpdateInterestMap_Event
    std::shared_ptr<graspnet_ros::srv::UpdateInterestMap_Event_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__graspnet_ros__srv__UpdateInterestMap_Event
    std::shared_ptr<graspnet_ros::srv::UpdateInterestMap_Event_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const UpdateInterestMap_Event_ & other) const
  {
    if (this->info != other.info) {
      return false;
    }
    if (this->request != other.request) {
      return false;
    }
    if (this->response != other.response) {
      return false;
    }
    return true;
  }
  bool operator!=(const UpdateInterestMap_Event_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct UpdateInterestMap_Event_

// alias to use template instance with default allocator
using UpdateInterestMap_Event =
  graspnet_ros::srv::UpdateInterestMap_Event_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace graspnet_ros

namespace graspnet_ros
{

namespace srv
{

struct UpdateInterestMap
{
  using Request = graspnet_ros::srv::UpdateInterestMap_Request;
  using Response = graspnet_ros::srv::UpdateInterestMap_Response;
  using Event = graspnet_ros::srv::UpdateInterestMap_Event;
};

}  // namespace srv

}  // namespace graspnet_ros

#endif  // GRASPNET_ROS__SRV__DETAIL__UPDATE_INTEREST_MAP__STRUCT_HPP_
