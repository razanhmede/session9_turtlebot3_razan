// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from navigation_robot_custom_interfaces:action/MeasureLapTime.idl
// generated code does not contain a copyright notice

#ifndef NAVIGATION_ROBOT_CUSTOM_INTERFACES__ACTION__DETAIL__MEASURE_LAP_TIME__BUILDER_HPP_
#define NAVIGATION_ROBOT_CUSTOM_INTERFACES__ACTION__DETAIL__MEASURE_LAP_TIME__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "navigation_robot_custom_interfaces/action/detail/measure_lap_time__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace navigation_robot_custom_interfaces
{

namespace action
{


}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::navigation_robot_custom_interfaces::action::MeasureLapTime_Goal>()
{
  return ::navigation_robot_custom_interfaces::action::MeasureLapTime_Goal(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace navigation_robot_custom_interfaces


namespace navigation_robot_custom_interfaces
{

namespace action
{

namespace builder
{

class Init_MeasureLapTime_Result_total_time
{
public:
  Init_MeasureLapTime_Result_total_time()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::navigation_robot_custom_interfaces::action::MeasureLapTime_Result total_time(::navigation_robot_custom_interfaces::action::MeasureLapTime_Result::_total_time_type arg)
  {
    msg_.total_time = std::move(arg);
    return std::move(msg_);
  }

private:
  ::navigation_robot_custom_interfaces::action::MeasureLapTime_Result msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::navigation_robot_custom_interfaces::action::MeasureLapTime_Result>()
{
  return navigation_robot_custom_interfaces::action::builder::Init_MeasureLapTime_Result_total_time();
}

}  // namespace navigation_robot_custom_interfaces


namespace navigation_robot_custom_interfaces
{

namespace action
{

namespace builder
{

class Init_MeasureLapTime_Feedback_elapsed_time
{
public:
  Init_MeasureLapTime_Feedback_elapsed_time()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::navigation_robot_custom_interfaces::action::MeasureLapTime_Feedback elapsed_time(::navigation_robot_custom_interfaces::action::MeasureLapTime_Feedback::_elapsed_time_type arg)
  {
    msg_.elapsed_time = std::move(arg);
    return std::move(msg_);
  }

private:
  ::navigation_robot_custom_interfaces::action::MeasureLapTime_Feedback msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::navigation_robot_custom_interfaces::action::MeasureLapTime_Feedback>()
{
  return navigation_robot_custom_interfaces::action::builder::Init_MeasureLapTime_Feedback_elapsed_time();
}

}  // namespace navigation_robot_custom_interfaces


namespace navigation_robot_custom_interfaces
{

namespace action
{

namespace builder
{

class Init_MeasureLapTime_SendGoal_Request_goal
{
public:
  explicit Init_MeasureLapTime_SendGoal_Request_goal(::navigation_robot_custom_interfaces::action::MeasureLapTime_SendGoal_Request & msg)
  : msg_(msg)
  {}
  ::navigation_robot_custom_interfaces::action::MeasureLapTime_SendGoal_Request goal(::navigation_robot_custom_interfaces::action::MeasureLapTime_SendGoal_Request::_goal_type arg)
  {
    msg_.goal = std::move(arg);
    return std::move(msg_);
  }

private:
  ::navigation_robot_custom_interfaces::action::MeasureLapTime_SendGoal_Request msg_;
};

class Init_MeasureLapTime_SendGoal_Request_goal_id
{
public:
  Init_MeasureLapTime_SendGoal_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MeasureLapTime_SendGoal_Request_goal goal_id(::navigation_robot_custom_interfaces::action::MeasureLapTime_SendGoal_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_MeasureLapTime_SendGoal_Request_goal(msg_);
  }

private:
  ::navigation_robot_custom_interfaces::action::MeasureLapTime_SendGoal_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::navigation_robot_custom_interfaces::action::MeasureLapTime_SendGoal_Request>()
{
  return navigation_robot_custom_interfaces::action::builder::Init_MeasureLapTime_SendGoal_Request_goal_id();
}

}  // namespace navigation_robot_custom_interfaces


namespace navigation_robot_custom_interfaces
{

namespace action
{

namespace builder
{

class Init_MeasureLapTime_SendGoal_Response_stamp
{
public:
  explicit Init_MeasureLapTime_SendGoal_Response_stamp(::navigation_robot_custom_interfaces::action::MeasureLapTime_SendGoal_Response & msg)
  : msg_(msg)
  {}
  ::navigation_robot_custom_interfaces::action::MeasureLapTime_SendGoal_Response stamp(::navigation_robot_custom_interfaces::action::MeasureLapTime_SendGoal_Response::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return std::move(msg_);
  }

private:
  ::navigation_robot_custom_interfaces::action::MeasureLapTime_SendGoal_Response msg_;
};

class Init_MeasureLapTime_SendGoal_Response_accepted
{
public:
  Init_MeasureLapTime_SendGoal_Response_accepted()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MeasureLapTime_SendGoal_Response_stamp accepted(::navigation_robot_custom_interfaces::action::MeasureLapTime_SendGoal_Response::_accepted_type arg)
  {
    msg_.accepted = std::move(arg);
    return Init_MeasureLapTime_SendGoal_Response_stamp(msg_);
  }

private:
  ::navigation_robot_custom_interfaces::action::MeasureLapTime_SendGoal_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::navigation_robot_custom_interfaces::action::MeasureLapTime_SendGoal_Response>()
{
  return navigation_robot_custom_interfaces::action::builder::Init_MeasureLapTime_SendGoal_Response_accepted();
}

}  // namespace navigation_robot_custom_interfaces


namespace navigation_robot_custom_interfaces
{

namespace action
{

namespace builder
{

class Init_MeasureLapTime_GetResult_Request_goal_id
{
public:
  Init_MeasureLapTime_GetResult_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::navigation_robot_custom_interfaces::action::MeasureLapTime_GetResult_Request goal_id(::navigation_robot_custom_interfaces::action::MeasureLapTime_GetResult_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::navigation_robot_custom_interfaces::action::MeasureLapTime_GetResult_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::navigation_robot_custom_interfaces::action::MeasureLapTime_GetResult_Request>()
{
  return navigation_robot_custom_interfaces::action::builder::Init_MeasureLapTime_GetResult_Request_goal_id();
}

}  // namespace navigation_robot_custom_interfaces


namespace navigation_robot_custom_interfaces
{

namespace action
{

namespace builder
{

class Init_MeasureLapTime_GetResult_Response_result
{
public:
  explicit Init_MeasureLapTime_GetResult_Response_result(::navigation_robot_custom_interfaces::action::MeasureLapTime_GetResult_Response & msg)
  : msg_(msg)
  {}
  ::navigation_robot_custom_interfaces::action::MeasureLapTime_GetResult_Response result(::navigation_robot_custom_interfaces::action::MeasureLapTime_GetResult_Response::_result_type arg)
  {
    msg_.result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::navigation_robot_custom_interfaces::action::MeasureLapTime_GetResult_Response msg_;
};

class Init_MeasureLapTime_GetResult_Response_status
{
public:
  Init_MeasureLapTime_GetResult_Response_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MeasureLapTime_GetResult_Response_result status(::navigation_robot_custom_interfaces::action::MeasureLapTime_GetResult_Response::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_MeasureLapTime_GetResult_Response_result(msg_);
  }

private:
  ::navigation_robot_custom_interfaces::action::MeasureLapTime_GetResult_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::navigation_robot_custom_interfaces::action::MeasureLapTime_GetResult_Response>()
{
  return navigation_robot_custom_interfaces::action::builder::Init_MeasureLapTime_GetResult_Response_status();
}

}  // namespace navigation_robot_custom_interfaces


namespace navigation_robot_custom_interfaces
{

namespace action
{

namespace builder
{

class Init_MeasureLapTime_FeedbackMessage_feedback
{
public:
  explicit Init_MeasureLapTime_FeedbackMessage_feedback(::navigation_robot_custom_interfaces::action::MeasureLapTime_FeedbackMessage & msg)
  : msg_(msg)
  {}
  ::navigation_robot_custom_interfaces::action::MeasureLapTime_FeedbackMessage feedback(::navigation_robot_custom_interfaces::action::MeasureLapTime_FeedbackMessage::_feedback_type arg)
  {
    msg_.feedback = std::move(arg);
    return std::move(msg_);
  }

private:
  ::navigation_robot_custom_interfaces::action::MeasureLapTime_FeedbackMessage msg_;
};

class Init_MeasureLapTime_FeedbackMessage_goal_id
{
public:
  Init_MeasureLapTime_FeedbackMessage_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MeasureLapTime_FeedbackMessage_feedback goal_id(::navigation_robot_custom_interfaces::action::MeasureLapTime_FeedbackMessage::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_MeasureLapTime_FeedbackMessage_feedback(msg_);
  }

private:
  ::navigation_robot_custom_interfaces::action::MeasureLapTime_FeedbackMessage msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::navigation_robot_custom_interfaces::action::MeasureLapTime_FeedbackMessage>()
{
  return navigation_robot_custom_interfaces::action::builder::Init_MeasureLapTime_FeedbackMessage_goal_id();
}

}  // namespace navigation_robot_custom_interfaces

#endif  // NAVIGATION_ROBOT_CUSTOM_INTERFACES__ACTION__DETAIL__MEASURE_LAP_TIME__BUILDER_HPP_
