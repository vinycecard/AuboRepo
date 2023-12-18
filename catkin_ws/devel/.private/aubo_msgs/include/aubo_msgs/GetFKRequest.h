// Generated by gencpp from file aubo_msgs/GetFKRequest.msg
// DO NOT EDIT!


#ifndef AUBO_MSGS_MESSAGE_GETFKREQUEST_H
#define AUBO_MSGS_MESSAGE_GETFKREQUEST_H


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
struct GetFKRequest_
{
  typedef GetFKRequest_<ContainerAllocator> Type;

  GetFKRequest_()
    : joint()  {
    }
  GetFKRequest_(const ContainerAllocator& _alloc)
    : joint(_alloc)  {
  (void)_alloc;
    }



   typedef std::vector<float, typename ContainerAllocator::template rebind<float>::other >  _joint_type;
  _joint_type joint;





  typedef boost::shared_ptr< ::aubo_msgs::GetFKRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::aubo_msgs::GetFKRequest_<ContainerAllocator> const> ConstPtr;

}; // struct GetFKRequest_

typedef ::aubo_msgs::GetFKRequest_<std::allocator<void> > GetFKRequest;

typedef boost::shared_ptr< ::aubo_msgs::GetFKRequest > GetFKRequestPtr;
typedef boost::shared_ptr< ::aubo_msgs::GetFKRequest const> GetFKRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::aubo_msgs::GetFKRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::aubo_msgs::GetFKRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace aubo_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'aubo_msgs': ['/home/nicolas/catkin_ws/src/aubo_robot/aubo_msgs/msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::aubo_msgs::GetFKRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::aubo_msgs::GetFKRequest_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::aubo_msgs::GetFKRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::aubo_msgs::GetFKRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::aubo_msgs::GetFKRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::aubo_msgs::GetFKRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::aubo_msgs::GetFKRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "edecb4b6fff50f927a908742515e167a";
  }

  static const char* value(const ::aubo_msgs::GetFKRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xedecb4b6fff50f92ULL;
  static const uint64_t static_value2 = 0x7a908742515e167aULL;
};

template<class ContainerAllocator>
struct DataType< ::aubo_msgs::GetFKRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "aubo_msgs/GetFKRequest";
  }

  static const char* value(const ::aubo_msgs::GetFKRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::aubo_msgs::GetFKRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float32[] joint\n\
";
  }

  static const char* value(const ::aubo_msgs::GetFKRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::aubo_msgs::GetFKRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.joint);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct GetFKRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::aubo_msgs::GetFKRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::aubo_msgs::GetFKRequest_<ContainerAllocator>& v)
  {
    s << indent << "joint[]" << std::endl;
    for (size_t i = 0; i < v.joint.size(); ++i)
    {
      s << indent << "  joint[" << i << "]: ";
      Printer<float>::stream(s, indent + "  ", v.joint[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // AUBO_MSGS_MESSAGE_GETFKREQUEST_H
