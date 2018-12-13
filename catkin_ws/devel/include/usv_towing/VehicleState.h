// Generated by gencpp from file usv_towing/VehicleState.msg
// DO NOT EDIT!


#ifndef USV_TOWING_MESSAGE_VEHICLESTATE_H
#define USV_TOWING_MESSAGE_VEHICLESTATE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Twist.h>

namespace usv_towing
{
template <class ContainerAllocator>
struct VehicleState_
{
  typedef VehicleState_<ContainerAllocator> Type;

  VehicleState_()
    : t(0.0)
    , pose()
    , vel()
    , acc()  {
    }
  VehicleState_(const ContainerAllocator& _alloc)
    : t(0.0)
    , pose(_alloc)
    , vel(_alloc)
    , acc(_alloc)  {
  (void)_alloc;
    }



   typedef double _t_type;
  _t_type t;

   typedef  ::geometry_msgs::Pose2D_<ContainerAllocator>  _pose_type;
  _pose_type pose;

   typedef  ::geometry_msgs::Twist_<ContainerAllocator>  _vel_type;
  _vel_type vel;

   typedef  ::geometry_msgs::Twist_<ContainerAllocator>  _acc_type;
  _acc_type acc;





  typedef boost::shared_ptr< ::usv_towing::VehicleState_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::usv_towing::VehicleState_<ContainerAllocator> const> ConstPtr;

}; // struct VehicleState_

typedef ::usv_towing::VehicleState_<std::allocator<void> > VehicleState;

typedef boost::shared_ptr< ::usv_towing::VehicleState > VehicleStatePtr;
typedef boost::shared_ptr< ::usv_towing::VehicleState const> VehicleStateConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::usv_towing::VehicleState_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::usv_towing::VehicleState_<ContainerAllocator> >::stream(s, "", v);
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
struct IsFixedSize< ::usv_towing::VehicleState_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::usv_towing::VehicleState_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::usv_towing::VehicleState_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::usv_towing::VehicleState_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::usv_towing::VehicleState_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::usv_towing::VehicleState_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::usv_towing::VehicleState_<ContainerAllocator> >
{
  static const char* value()
  {
    return "267dc2162d9371f64d61a71605f7ca88";
  }

  static const char* value(const ::usv_towing::VehicleState_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x267dc2162d9371f6ULL;
  static const uint64_t static_value2 = 0x4d61a71605f7ca88ULL;
};

template<class ContainerAllocator>
struct DataType< ::usv_towing::VehicleState_<ContainerAllocator> >
{
  static const char* value()
  {
    return "usv_towing/VehicleState";
  }

  static const char* value(const ::usv_towing::VehicleState_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::usv_towing::VehicleState_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float64 t\n\
geometry_msgs/Pose2D pose\n\
geometry_msgs/Twist vel\n\
geometry_msgs/Twist acc\n\
================================================================================\n\
MSG: geometry_msgs/Pose2D\n\
# This expresses a position and orientation on a 2D manifold.\n\
\n\
float64 x\n\
float64 y\n\
float64 theta\n\
================================================================================\n\
MSG: geometry_msgs/Twist\n\
# This expresses velocity in free space broken into its linear and angular parts.\n\
Vector3  linear\n\
Vector3  angular\n\
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

  static const char* value(const ::usv_towing::VehicleState_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::usv_towing::VehicleState_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.t);
      stream.next(m.pose);
      stream.next(m.vel);
      stream.next(m.acc);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct VehicleState_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::usv_towing::VehicleState_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::usv_towing::VehicleState_<ContainerAllocator>& v)
  {
    s << indent << "t: ";
    Printer<double>::stream(s, indent + "  ", v.t);
    s << indent << "pose: ";
    s << std::endl;
    Printer< ::geometry_msgs::Pose2D_<ContainerAllocator> >::stream(s, indent + "  ", v.pose);
    s << indent << "vel: ";
    s << std::endl;
    Printer< ::geometry_msgs::Twist_<ContainerAllocator> >::stream(s, indent + "  ", v.vel);
    s << indent << "acc: ";
    s << std::endl;
    Printer< ::geometry_msgs::Twist_<ContainerAllocator> >::stream(s, indent + "  ", v.acc);
  }
};

} // namespace message_operations
} // namespace ros

#endif // USV_TOWING_MESSAGE_VEHICLESTATE_H
