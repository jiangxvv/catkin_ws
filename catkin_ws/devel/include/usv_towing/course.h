// Generated by gencpp from file usv_towing/course.msg
// DO NOT EDIT!


#ifndef USV_TOWING_MESSAGE_COURSE_H
#define USV_TOWING_MESSAGE_COURSE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Vector3.h>

namespace usv_towing
{
template <class ContainerAllocator>
struct course_
{
  typedef course_<ContainerAllocator> Type;

  course_()
    : pose()
    , velocity()
    , acceleration()  {
    }
  course_(const ContainerAllocator& _alloc)
    : pose(_alloc)
    , velocity(_alloc)
    , acceleration(_alloc)  {
  (void)_alloc;
    }



   typedef  ::geometry_msgs::Vector3_<ContainerAllocator>  _pose_type;
  _pose_type pose;

   typedef  ::geometry_msgs::Vector3_<ContainerAllocator>  _velocity_type;
  _velocity_type velocity;

   typedef  ::geometry_msgs::Vector3_<ContainerAllocator>  _acceleration_type;
  _acceleration_type acceleration;





  typedef boost::shared_ptr< ::usv_towing::course_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::usv_towing::course_<ContainerAllocator> const> ConstPtr;

}; // struct course_

typedef ::usv_towing::course_<std::allocator<void> > course;

typedef boost::shared_ptr< ::usv_towing::course > coursePtr;
typedef boost::shared_ptr< ::usv_towing::course const> courseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::usv_towing::course_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::usv_towing::course_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace usv_towing

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'geometry_msgs': ['/opt/ros/kinetic/share/geometry_msgs/cmake/../msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'usv_towing': ['/home/jiangxvv/catkin_ws/src/catkin_ws/src/usv_towing/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::usv_towing::course_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::usv_towing::course_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::usv_towing::course_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::usv_towing::course_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::usv_towing::course_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::usv_towing::course_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::usv_towing::course_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ee91ad718657acbe3311fc42870a477e";
  }

  static const char* value(const ::usv_towing::course_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xee91ad718657acbeULL;
  static const uint64_t static_value2 = 0x3311fc42870a477eULL;
};

template<class ContainerAllocator>
struct DataType< ::usv_towing::course_<ContainerAllocator> >
{
  static const char* value()
  {
    return "usv_towing/course";
  }

  static const char* value(const ::usv_towing::course_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::usv_towing::course_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# course\n\
geometry_msgs/Vector3 pose\n\
geometry_msgs/Vector3 velocity\n\
geometry_msgs/Vector3 acceleration\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Vector3\n\
# This represents a vector in free space. \n\
# It is only meant to represent a direction. Therefore, it does not\n\
# make sense to apply a translation to it (e.g., when applying a \n\
# generic rigid transformation to a Vector3, tf2 will only apply the\n\
# rotation). If you want your data to be translatable too, use the\n\
# geometry_msgs/Point message instead.\n\
\n\
float64 x\n\
float64 y\n\
float64 z\n\
";
  }

  static const char* value(const ::usv_towing::course_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::usv_towing::course_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.pose);
      stream.next(m.velocity);
      stream.next(m.acceleration);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct course_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::usv_towing::course_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::usv_towing::course_<ContainerAllocator>& v)
  {
    s << indent << "pose: ";
    s << std::endl;
    Printer< ::geometry_msgs::Vector3_<ContainerAllocator> >::stream(s, indent + "  ", v.pose);
    s << indent << "velocity: ";
    s << std::endl;
    Printer< ::geometry_msgs::Vector3_<ContainerAllocator> >::stream(s, indent + "  ", v.velocity);
    s << indent << "acceleration: ";
    s << std::endl;
    Printer< ::geometry_msgs::Vector3_<ContainerAllocator> >::stream(s, indent + "  ", v.acceleration);
  }
};

} // namespace message_operations
} // namespace ros

#endif // USV_TOWING_MESSAGE_COURSE_H
