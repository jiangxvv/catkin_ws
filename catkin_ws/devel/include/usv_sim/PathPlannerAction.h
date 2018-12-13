// Generated by gencpp from file usv_sim/PathPlannerAction.msg
// DO NOT EDIT!


#ifndef USV_SIM_MESSAGE_PATHPLANNERACTION_H
#define USV_SIM_MESSAGE_PATHPLANNERACTION_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <usv_sim/PathPlannerActionGoal.h>
#include <usv_sim/PathPlannerActionResult.h>
#include <usv_sim/PathPlannerActionFeedback.h>

namespace usv_sim
{
template <class ContainerAllocator>
struct PathPlannerAction_
{
  typedef PathPlannerAction_<ContainerAllocator> Type;

  PathPlannerAction_()
    : action_goal()
    , action_result()
    , action_feedback()  {
    }
  PathPlannerAction_(const ContainerAllocator& _alloc)
    : action_goal(_alloc)
    , action_result(_alloc)
    , action_feedback(_alloc)  {
  (void)_alloc;
    }



   typedef  ::usv_sim::PathPlannerActionGoal_<ContainerAllocator>  _action_goal_type;
  _action_goal_type action_goal;

   typedef  ::usv_sim::PathPlannerActionResult_<ContainerAllocator>  _action_result_type;
  _action_result_type action_result;

   typedef  ::usv_sim::PathPlannerActionFeedback_<ContainerAllocator>  _action_feedback_type;
  _action_feedback_type action_feedback;





  typedef boost::shared_ptr< ::usv_sim::PathPlannerAction_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::usv_sim::PathPlannerAction_<ContainerAllocator> const> ConstPtr;

}; // struct PathPlannerAction_

typedef ::usv_sim::PathPlannerAction_<std::allocator<void> > PathPlannerAction;

typedef boost::shared_ptr< ::usv_sim::PathPlannerAction > PathPlannerActionPtr;
typedef boost::shared_ptr< ::usv_sim::PathPlannerAction const> PathPlannerActionConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::usv_sim::PathPlannerAction_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::usv_sim::PathPlannerAction_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace usv_sim

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'usv_sim': ['/home/jiangxvv/catkin_ws/src/catkin_ws/src/bobli1987-autonomous_tugboat-a80163bcc2ef/usv_sim/msg', '/home/jiangxvv/catkin_ws/devel/share/usv_sim/msg'], 'actionlib_msgs': ['/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/kinetic/share/geometry_msgs/cmake/../msg'], 'visualization_msgs': ['/opt/ros/kinetic/share/visualization_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::usv_sim::PathPlannerAction_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::usv_sim::PathPlannerAction_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::usv_sim::PathPlannerAction_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::usv_sim::PathPlannerAction_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::usv_sim::PathPlannerAction_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::usv_sim::PathPlannerAction_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::usv_sim::PathPlannerAction_<ContainerAllocator> >
{
  static const char* value()
  {
    return "afdf263ffe9f8a1d69366b74567e7b53";
  }

  static const char* value(const ::usv_sim::PathPlannerAction_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xafdf263ffe9f8a1dULL;
  static const uint64_t static_value2 = 0x69366b74567e7b53ULL;
};

template<class ContainerAllocator>
struct DataType< ::usv_sim::PathPlannerAction_<ContainerAllocator> >
{
  static const char* value()
  {
    return "usv_sim/PathPlannerAction";
  }

  static const char* value(const ::usv_sim::PathPlannerAction_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::usv_sim::PathPlannerAction_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
\n\
PathPlannerActionGoal action_goal\n\
PathPlannerActionResult action_result\n\
PathPlannerActionFeedback action_feedback\n\
\n\
================================================================================\n\
MSG: usv_sim/PathPlannerActionGoal\n\
# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
\n\
Header header\n\
actionlib_msgs/GoalID goal_id\n\
PathPlannerGoal goal\n\
\n\
================================================================================\n\
MSG: std_msgs/Header\n\
# Standard metadata for higher-level stamped data types.\n\
# This is generally used to communicate timestamped data \n\
# in a particular coordinate frame.\n\
# \n\
# sequence ID: consecutively increasing ID \n\
uint32 seq\n\
#Two-integer timestamp that is expressed as:\n\
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n\
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n\
# time-handling sugar is provided by the client library\n\
time stamp\n\
#Frame this data is associated with\n\
# 0: no frame\n\
# 1: global frame\n\
string frame_id\n\
\n\
================================================================================\n\
MSG: actionlib_msgs/GoalID\n\
# The stamp should store the time at which this goal was requested.\n\
# It is used by an action server when it tries to preempt all\n\
# goals that were requested before a certain time\n\
time stamp\n\
\n\
# The id provides a way to associate feedback and\n\
# result message with specific goal requests. The id\n\
# specified must be unique.\n\
string id\n\
\n\
\n\
================================================================================\n\
MSG: usv_sim/PathPlannerGoal\n\
# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
# goal definition\n\
float64[] start_pose\n\
float64[] start_vel\n\
float64[] end_pose\n\
\n\
================================================================================\n\
MSG: usv_sim/PathPlannerActionResult\n\
# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
\n\
Header header\n\
actionlib_msgs/GoalStatus status\n\
PathPlannerResult result\n\
\n\
================================================================================\n\
MSG: actionlib_msgs/GoalStatus\n\
GoalID goal_id\n\
uint8 status\n\
uint8 PENDING         = 0   # The goal has yet to be processed by the action server\n\
uint8 ACTIVE          = 1   # The goal is currently being processed by the action server\n\
uint8 PREEMPTED       = 2   # The goal received a cancel request after it started executing\n\
                            #   and has since completed its execution (Terminal State)\n\
uint8 SUCCEEDED       = 3   # The goal was achieved successfully by the action server (Terminal State)\n\
uint8 ABORTED         = 4   # The goal was aborted during execution by the action server due\n\
                            #    to some failure (Terminal State)\n\
uint8 REJECTED        = 5   # The goal was rejected by the action server without being processed,\n\
                            #    because the goal was unattainable or invalid (Terminal State)\n\
uint8 PREEMPTING      = 6   # The goal received a cancel request after it started executing\n\
                            #    and has not yet completed execution\n\
uint8 RECALLING       = 7   # The goal received a cancel request before it started executing,\n\
                            #    but the action server has not yet confirmed that the goal is canceled\n\
uint8 RECALLED        = 8   # The goal received a cancel request before it started executing\n\
                            #    and was successfully cancelled (Terminal State)\n\
uint8 LOST            = 9   # An action client can determine that a goal is LOST. This should not be\n\
                            #    sent over the wire by an action server\n\
\n\
#Allow for the user to associate a string with GoalStatus for debugging\n\
string text\n\
\n\
\n\
================================================================================\n\
MSG: usv_sim/PathPlannerResult\n\
# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
# result definition\n\
float64[] wpt_x\n\
float64[] wpt_y\n\
float64[] wpt_psi\n\
\n\
================================================================================\n\
MSG: usv_sim/PathPlannerActionFeedback\n\
# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
\n\
Header header\n\
actionlib_msgs/GoalStatus status\n\
PathPlannerFeedback feedback\n\
\n\
================================================================================\n\
MSG: usv_sim/PathPlannerFeedback\n\
# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
# feedback definition\n\
float64 elapsed_time\n\
\n\
";
  }

  static const char* value(const ::usv_sim::PathPlannerAction_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::usv_sim::PathPlannerAction_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.action_goal);
      stream.next(m.action_result);
      stream.next(m.action_feedback);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct PathPlannerAction_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::usv_sim::PathPlannerAction_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::usv_sim::PathPlannerAction_<ContainerAllocator>& v)
  {
    s << indent << "action_goal: ";
    s << std::endl;
    Printer< ::usv_sim::PathPlannerActionGoal_<ContainerAllocator> >::stream(s, indent + "  ", v.action_goal);
    s << indent << "action_result: ";
    s << std::endl;
    Printer< ::usv_sim::PathPlannerActionResult_<ContainerAllocator> >::stream(s, indent + "  ", v.action_result);
    s << indent << "action_feedback: ";
    s << std::endl;
    Printer< ::usv_sim::PathPlannerActionFeedback_<ContainerAllocator> >::stream(s, indent + "  ", v.action_feedback);
  }
};

} // namespace message_operations
} // namespace ros

#endif // USV_SIM_MESSAGE_PATHPLANNERACTION_H