// Generated by gencpp from file usv_sim/WaypointTrackingFeedback.msg
// DO NOT EDIT!


#ifndef USV_SIM_MESSAGE_WAYPOINTTRACKINGFEEDBACK_H
#define USV_SIM_MESSAGE_WAYPOINTTRACKINGFEEDBACK_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace usv_sim
{
template <class ContainerAllocator>
struct WaypointTrackingFeedback_
{
  typedef WaypointTrackingFeedback_<ContainerAllocator> Type;

  WaypointTrackingFeedback_()
    : progress(0)  {
    }
  WaypointTrackingFeedback_(const ContainerAllocator& _alloc)
    : progress(0)  {
  (void)_alloc;
    }



   typedef uint32_t _progress_type;
  _progress_type progress;





  typedef boost::shared_ptr< ::usv_sim::WaypointTrackingFeedback_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::usv_sim::WaypointTrackingFeedback_<ContainerAllocator> const> ConstPtr;

}; // struct WaypointTrackingFeedback_

typedef ::usv_sim::WaypointTrackingFeedback_<std::allocator<void> > WaypointTrackingFeedback;

typedef boost::shared_ptr< ::usv_sim::WaypointTrackingFeedback > WaypointTrackingFeedbackPtr;
typedef boost::shared_ptr< ::usv_sim::WaypointTrackingFeedback const> WaypointTrackingFeedbackConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::usv_sim::WaypointTrackingFeedback_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::usv_sim::WaypointTrackingFeedback_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace usv_sim

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'usv_sim': ['/home/jiangxvv/catkin_ws/src/catkin_ws/src/bobli1987-autonomous_tugboat-a80163bcc2ef/usv_sim/msg', '/home/jiangxvv/catkin_ws/devel/share/usv_sim/msg'], 'actionlib_msgs': ['/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/kinetic/share/geometry_msgs/cmake/../msg'], 'visualization_msgs': ['/opt/ros/kinetic/share/visualization_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::usv_sim::WaypointTrackingFeedback_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::usv_sim::WaypointTrackingFeedback_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::usv_sim::WaypointTrackingFeedback_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::usv_sim::WaypointTrackingFeedback_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::usv_sim::WaypointTrackingFeedback_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::usv_sim::WaypointTrackingFeedback_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::usv_sim::WaypointTrackingFeedback_<ContainerAllocator> >
{
  static const char* value()
  {
    return "6d745b453082f511ce5401257d5f4a34";
  }

  static const char* value(const ::usv_sim::WaypointTrackingFeedback_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x6d745b453082f511ULL;
  static const uint64_t static_value2 = 0xce5401257d5f4a34ULL;
};

template<class ContainerAllocator>
struct DataType< ::usv_sim::WaypointTrackingFeedback_<ContainerAllocator> >
{
  static const char* value()
  {
    return "usv_sim/WaypointTrackingFeedback";
  }

  static const char* value(const ::usv_sim::WaypointTrackingFeedback_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::usv_sim::WaypointTrackingFeedback_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
# feedback definition\n\
uint32 progress\n\
\n\
";
  }

  static const char* value(const ::usv_sim::WaypointTrackingFeedback_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::usv_sim::WaypointTrackingFeedback_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.progress);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct WaypointTrackingFeedback_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::usv_sim::WaypointTrackingFeedback_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::usv_sim::WaypointTrackingFeedback_<ContainerAllocator>& v)
  {
    s << indent << "progress: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.progress);
  }
};

} // namespace message_operations
} // namespace ros

#endif // USV_SIM_MESSAGE_WAYPOINTTRACKINGFEEDBACK_H