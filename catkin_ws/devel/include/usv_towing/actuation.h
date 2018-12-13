// Generated by gencpp from file usv_towing/actuation.msg
// DO NOT EDIT!


#ifndef USV_TOWING_MESSAGE_ACTUATION_H
#define USV_TOWING_MESSAGE_ACTUATION_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace usv_towing
{
template <class ContainerAllocator>
struct actuation_
{
  typedef actuation_<ContainerAllocator> Type;

  actuation_()
    : tug1(0.0)
    , tug2(0.0)
    , tug3(0.0)
    , tug4(0.0)  {
    }
  actuation_(const ContainerAllocator& _alloc)
    : tug1(0.0)
    , tug2(0.0)
    , tug3(0.0)
    , tug4(0.0)  {
  (void)_alloc;
    }



   typedef double _tug1_type;
  _tug1_type tug1;

   typedef double _tug2_type;
  _tug2_type tug2;

   typedef double _tug3_type;
  _tug3_type tug3;

   typedef double _tug4_type;
  _tug4_type tug4;





  typedef boost::shared_ptr< ::usv_towing::actuation_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::usv_towing::actuation_<ContainerAllocator> const> ConstPtr;

}; // struct actuation_

typedef ::usv_towing::actuation_<std::allocator<void> > actuation;

typedef boost::shared_ptr< ::usv_towing::actuation > actuationPtr;
typedef boost::shared_ptr< ::usv_towing::actuation const> actuationConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::usv_towing::actuation_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::usv_towing::actuation_<ContainerAllocator> >::stream(s, "", v);
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
struct IsFixedSize< ::usv_towing::actuation_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::usv_towing::actuation_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::usv_towing::actuation_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::usv_towing::actuation_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::usv_towing::actuation_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::usv_towing::actuation_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::usv_towing::actuation_<ContainerAllocator> >
{
  static const char* value()
  {
    return "6d5d273aeba236b992836cce53bb09da";
  }

  static const char* value(const ::usv_towing::actuation_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x6d5d273aeba236b9ULL;
  static const uint64_t static_value2 = 0x92836cce53bb09daULL;
};

template<class ContainerAllocator>
struct DataType< ::usv_towing::actuation_<ContainerAllocator> >
{
  static const char* value()
  {
    return "usv_towing/actuation";
  }

  static const char* value(const ::usv_towing::actuation_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::usv_towing::actuation_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float64 tug1\n\
float64 tug2\n\
float64 tug3\n\
float64 tug4\n\
";
  }

  static const char* value(const ::usv_towing::actuation_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::usv_towing::actuation_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.tug1);
      stream.next(m.tug2);
      stream.next(m.tug3);
      stream.next(m.tug4);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct actuation_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::usv_towing::actuation_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::usv_towing::actuation_<ContainerAllocator>& v)
  {
    s << indent << "tug1: ";
    Printer<double>::stream(s, indent + "  ", v.tug1);
    s << indent << "tug2: ";
    Printer<double>::stream(s, indent + "  ", v.tug2);
    s << indent << "tug3: ";
    Printer<double>::stream(s, indent + "  ", v.tug3);
    s << indent << "tug4: ";
    Printer<double>::stream(s, indent + "  ", v.tug4);
  }
};

} // namespace message_operations
} // namespace ros

#endif // USV_TOWING_MESSAGE_ACTUATION_H