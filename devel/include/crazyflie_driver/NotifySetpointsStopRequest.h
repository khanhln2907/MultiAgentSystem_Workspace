// Generated by gencpp from file crazyflie_driver/NotifySetpointsStopRequest.msg
// DO NOT EDIT!


#ifndef CRAZYFLIE_DRIVER_MESSAGE_NOTIFYSETPOINTSSTOPREQUEST_H
#define CRAZYFLIE_DRIVER_MESSAGE_NOTIFYSETPOINTSSTOPREQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace crazyflie_driver
{
template <class ContainerAllocator>
struct NotifySetpointsStopRequest_
{
  typedef NotifySetpointsStopRequest_<ContainerAllocator> Type;

  NotifySetpointsStopRequest_()
    : groupMask(0)
    , remainValidMillisecs(0)  {
    }
  NotifySetpointsStopRequest_(const ContainerAllocator& _alloc)
    : groupMask(0)
    , remainValidMillisecs(0)  {
  (void)_alloc;
    }



   typedef uint8_t _groupMask_type;
  _groupMask_type groupMask;

   typedef uint32_t _remainValidMillisecs_type;
  _remainValidMillisecs_type remainValidMillisecs;





  typedef boost::shared_ptr< ::crazyflie_driver::NotifySetpointsStopRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::crazyflie_driver::NotifySetpointsStopRequest_<ContainerAllocator> const> ConstPtr;

}; // struct NotifySetpointsStopRequest_

typedef ::crazyflie_driver::NotifySetpointsStopRequest_<std::allocator<void> > NotifySetpointsStopRequest;

typedef boost::shared_ptr< ::crazyflie_driver::NotifySetpointsStopRequest > NotifySetpointsStopRequestPtr;
typedef boost::shared_ptr< ::crazyflie_driver::NotifySetpointsStopRequest const> NotifySetpointsStopRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::crazyflie_driver::NotifySetpointsStopRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::crazyflie_driver::NotifySetpointsStopRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace crazyflie_driver

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'crazyflie_driver': ['/home/qingchen/catkin_ws/src/crazyflie_ros/crazyflie_driver/msg'], 'geometry_msgs': ['/opt/ros/kinetic/share/geometry_msgs/cmake/../msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::crazyflie_driver::NotifySetpointsStopRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::crazyflie_driver::NotifySetpointsStopRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::crazyflie_driver::NotifySetpointsStopRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::crazyflie_driver::NotifySetpointsStopRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::crazyflie_driver::NotifySetpointsStopRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::crazyflie_driver::NotifySetpointsStopRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::crazyflie_driver::NotifySetpointsStopRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "5e1e40fcf516d3883bc228627d3e1e43";
  }

  static const char* value(const ::crazyflie_driver::NotifySetpointsStopRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x5e1e40fcf516d388ULL;
  static const uint64_t static_value2 = 0x3bc228627d3e1e43ULL;
};

template<class ContainerAllocator>
struct DataType< ::crazyflie_driver::NotifySetpointsStopRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "crazyflie_driver/NotifySetpointsStopRequest";
  }

  static const char* value(const ::crazyflie_driver::NotifySetpointsStopRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::crazyflie_driver::NotifySetpointsStopRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "uint8 groupMask\n\
uint32 remainValidMillisecs\n\
";
  }

  static const char* value(const ::crazyflie_driver::NotifySetpointsStopRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::crazyflie_driver::NotifySetpointsStopRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.groupMask);
      stream.next(m.remainValidMillisecs);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct NotifySetpointsStopRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::crazyflie_driver::NotifySetpointsStopRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::crazyflie_driver::NotifySetpointsStopRequest_<ContainerAllocator>& v)
  {
    s << indent << "groupMask: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.groupMask);
    s << indent << "remainValidMillisecs: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.remainValidMillisecs);
  }
};

} // namespace message_operations
} // namespace ros

#endif // CRAZYFLIE_DRIVER_MESSAGE_NOTIFYSETPOINTSSTOPREQUEST_H
