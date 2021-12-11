// Generated by gencpp from file tip/UnicycleInfoStruct.msg
// DO NOT EDIT!


#ifndef TIP_MESSAGE_UNICYCLEINFOSTRUCT_H
#define TIP_MESSAGE_UNICYCLEINFOSTRUCT_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>

namespace tip
{
template <class ContainerAllocator>
struct UnicycleInfoStruct_
{
  typedef UnicycleInfoStruct_<ContainerAllocator> Type;

  UnicycleInfoStruct_()
    : header()
    , TransmitterID(0)
    , AgentPosX(0.0)
    , AgentPosY(0.0)
    , AgentTheta(0.0)
    , VirtualCenterX(0.0)
    , VirtualCenterY(0.0)
    , V_BLF(0.0)  {
    }
  UnicycleInfoStruct_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , TransmitterID(0)
    , AgentPosX(0.0)
    , AgentPosY(0.0)
    , AgentTheta(0.0)
    , VirtualCenterX(0.0)
    , VirtualCenterY(0.0)
    , V_BLF(0.0)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef int32_t _TransmitterID_type;
  _TransmitterID_type TransmitterID;

   typedef float _AgentPosX_type;
  _AgentPosX_type AgentPosX;

   typedef float _AgentPosY_type;
  _AgentPosY_type AgentPosY;

   typedef float _AgentTheta_type;
  _AgentTheta_type AgentTheta;

   typedef float _VirtualCenterX_type;
  _VirtualCenterX_type VirtualCenterX;

   typedef float _VirtualCenterY_type;
  _VirtualCenterY_type VirtualCenterY;

   typedef float _V_BLF_type;
  _V_BLF_type V_BLF;





  typedef boost::shared_ptr< ::tip::UnicycleInfoStruct_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::tip::UnicycleInfoStruct_<ContainerAllocator> const> ConstPtr;

}; // struct UnicycleInfoStruct_

typedef ::tip::UnicycleInfoStruct_<std::allocator<void> > UnicycleInfoStruct;

typedef boost::shared_ptr< ::tip::UnicycleInfoStruct > UnicycleInfoStructPtr;
typedef boost::shared_ptr< ::tip::UnicycleInfoStruct const> UnicycleInfoStructConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::tip::UnicycleInfoStruct_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::tip::UnicycleInfoStruct_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace tip

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': True}
// {'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/kinetic/share/geometry_msgs/cmake/../msg'], 'tip': ['/home/qingchen/catkin_ws/src/tip/msg'], 'qualisys': ['/home/qingchen/catkin_ws/src/qualisys/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::tip::UnicycleInfoStruct_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::tip::UnicycleInfoStruct_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::tip::UnicycleInfoStruct_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::tip::UnicycleInfoStruct_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::tip::UnicycleInfoStruct_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::tip::UnicycleInfoStruct_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::tip::UnicycleInfoStruct_<ContainerAllocator> >
{
  static const char* value()
  {
    return "5cea3926dc7b0a30646b151951d6b5f5";
  }

  static const char* value(const ::tip::UnicycleInfoStruct_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x5cea3926dc7b0a30ULL;
  static const uint64_t static_value2 = 0x646b151951d6b5f5ULL;
};

template<class ContainerAllocator>
struct DataType< ::tip::UnicycleInfoStruct_<ContainerAllocator> >
{
  static const char* value()
  {
    return "tip/UnicycleInfoStruct";
  }

  static const char* value(const ::tip::UnicycleInfoStruct_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::tip::UnicycleInfoStruct_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Header header\n\
int32 TransmitterID\n\
float32 AgentPosX\n\
float32 AgentPosY\n\
float32 AgentTheta\n\
float32 VirtualCenterX\n\
float32 VirtualCenterY\n\
float32 V_BLF\n\
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
";
  }

  static const char* value(const ::tip::UnicycleInfoStruct_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::tip::UnicycleInfoStruct_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.TransmitterID);
      stream.next(m.AgentPosX);
      stream.next(m.AgentPosY);
      stream.next(m.AgentTheta);
      stream.next(m.VirtualCenterX);
      stream.next(m.VirtualCenterY);
      stream.next(m.V_BLF);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct UnicycleInfoStruct_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::tip::UnicycleInfoStruct_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::tip::UnicycleInfoStruct_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "TransmitterID: ";
    Printer<int32_t>::stream(s, indent + "  ", v.TransmitterID);
    s << indent << "AgentPosX: ";
    Printer<float>::stream(s, indent + "  ", v.AgentPosX);
    s << indent << "AgentPosY: ";
    Printer<float>::stream(s, indent + "  ", v.AgentPosY);
    s << indent << "AgentTheta: ";
    Printer<float>::stream(s, indent + "  ", v.AgentTheta);
    s << indent << "VirtualCenterX: ";
    Printer<float>::stream(s, indent + "  ", v.VirtualCenterX);
    s << indent << "VirtualCenterY: ";
    Printer<float>::stream(s, indent + "  ", v.VirtualCenterY);
    s << indent << "V_BLF: ";
    Printer<float>::stream(s, indent + "  ", v.V_BLF);
  }
};

} // namespace message_operations
} // namespace ros

#endif // TIP_MESSAGE_UNICYCLEINFOSTRUCT_H