// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from navigation_robot_custom_interfaces:srv/FindClosestWall.idl
// generated code does not contain a copyright notice

#ifndef NAVIGATION_ROBOT_CUSTOM_INTERFACES__SRV__DETAIL__FIND_CLOSEST_WALL__BUILDER_HPP_
#define NAVIGATION_ROBOT_CUSTOM_INTERFACES__SRV__DETAIL__FIND_CLOSEST_WALL__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "navigation_robot_custom_interfaces/srv/detail/find_closest_wall__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace navigation_robot_custom_interfaces
{

namespace srv
{


}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::navigation_robot_custom_interfaces::srv::FindClosestWall_Request>()
{
  return ::navigation_robot_custom_interfaces::srv::FindClosestWall_Request(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace navigation_robot_custom_interfaces


namespace navigation_robot_custom_interfaces
{

namespace srv
{

namespace builder
{

class Init_FindClosestWall_Response_z
{
public:
  explicit Init_FindClosestWall_Response_z(::navigation_robot_custom_interfaces::srv::FindClosestWall_Response & msg)
  : msg_(msg)
  {}
  ::navigation_robot_custom_interfaces::srv::FindClosestWall_Response z(::navigation_robot_custom_interfaces::srv::FindClosestWall_Response::_z_type arg)
  {
    msg_.z = std::move(arg);
    return std::move(msg_);
  }

private:
  ::navigation_robot_custom_interfaces::srv::FindClosestWall_Response msg_;
};

class Init_FindClosestWall_Response_y
{
public:
  explicit Init_FindClosestWall_Response_y(::navigation_robot_custom_interfaces::srv::FindClosestWall_Response & msg)
  : msg_(msg)
  {}
  Init_FindClosestWall_Response_z y(::navigation_robot_custom_interfaces::srv::FindClosestWall_Response::_y_type arg)
  {
    msg_.y = std::move(arg);
    return Init_FindClosestWall_Response_z(msg_);
  }

private:
  ::navigation_robot_custom_interfaces::srv::FindClosestWall_Response msg_;
};

class Init_FindClosestWall_Response_x
{
public:
  Init_FindClosestWall_Response_x()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_FindClosestWall_Response_y x(::navigation_robot_custom_interfaces::srv::FindClosestWall_Response::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_FindClosestWall_Response_y(msg_);
  }

private:
  ::navigation_robot_custom_interfaces::srv::FindClosestWall_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::navigation_robot_custom_interfaces::srv::FindClosestWall_Response>()
{
  return navigation_robot_custom_interfaces::srv::builder::Init_FindClosestWall_Response_x();
}

}  // namespace navigation_robot_custom_interfaces

#endif  // NAVIGATION_ROBOT_CUSTOM_INTERFACES__SRV__DETAIL__FIND_CLOSEST_WALL__BUILDER_HPP_
