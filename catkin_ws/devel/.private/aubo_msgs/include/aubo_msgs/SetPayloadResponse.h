// Generated by gencpp from file aubo_msgs/SetPayloadResponse.msg
// DO NOT EDIT!


#ifndef AUBO_MSGS_MESSAGE_SETPAYLOADRESPONSE_H
#define AUBO_MSGS_MESSAGE_SETPAYLOADRESPONSE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace aubo_msgs
{
template <class ContainerAllocator>
struct SetPayloadResponse_
{
  typedef SetPayloadResponse_<ContainerAllocator> Type;

  SetPayloadResponse_()
    : success(false)  {
    }
  SetPayloadResponse_(const ContainerAllocator& _alloc)
    : success(false)  {
  (void)_alloc;
    }



   typedef uint8_t _success_type;
  _success_type success;





  typedef boost::shared_ptr< ::aubo_msgs::SetPayloadResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::aubo_msgs::SetPayloadResponse_<ContainerAllocator> const> ConstPtr;

}; // struct SetPayloadResponse_

typedef ::aubo_msgs::SetPayloadResponse_<std::allocator<void> > SetPayloadResponse;

typedef boost::shared_ptr< ::aubo_msgs::SetPayloadResponse > SetPayloadResponsePtr;
typedef boost::shared_ptr< ::aubo_msgs::SetPayloadResponse const> SetPayloadResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::aubo_msgs::SetPayloadResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::aubo_msgs::SetPayloadResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace aubo_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'aubo_msgs': ['/home/nicolas/catkin_ws/src/aubo_robot/aubo_msgs/msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::aubo_msgs::SetPayloadResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::aubo_msgs::SetPayloadResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::aubo_msgs::SetPayloadResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::aubo_msgs::SetPayloadResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::aubo_msgs::SetPayloadResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::aubo_msgs::SetPayloadResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::aubo_msgs::SetPayloadResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "358e233cde0c8a8bcfea4ce193f8fc15";
  }

  static const char* value(const ::aubo_msgs::SetPayloadResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x358e233cde0c8a8bULL;
  static const uint64_t static_value2 = 0xcfea4ce193f8fc15ULL;
};

template<class ContainerAllocator>
struct DataType< ::aubo_msgs::SetPayloadResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "aubo_msgs/SetPayloadResponse";
  }

  static const char* value(const ::aubo_msgs::SetPayloadResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::aubo_msgs::SetPayloadResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "bool success\n\
\n\
";
  }

  static const char* value(const ::aubo_msgs::SetPayloadResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::aubo_msgs::SetPayloadResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.success);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct SetPayloadResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::aubo_msgs::SetPayloadResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::aubo_msgs::SetPayloadResponse_<ContainerAllocator>& v)
  {
    s << indent << "success: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.success);
  }
};

} // namespace message_operations
} // namespace ros

#endif // AUBO_MSGS_MESSAGE_SETPAYLOADRESPONSE_H
