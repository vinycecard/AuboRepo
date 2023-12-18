// Generated by gencpp from file aubo_msgs/GetIKRequest.msg
// DO NOT EDIT!


#ifndef AUBO_MSGS_MESSAGE_GETIKREQUEST_H
#define AUBO_MSGS_MESSAGE_GETIKREQUEST_H


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
struct GetIKRequest_
{
  typedef GetIKRequest_<ContainerAllocator> Type;

  GetIKRequest_()
    : ref_joint()
    , pos()
    , ori()  {
    }
  GetIKRequest_(const ContainerAllocator& _alloc)
    : ref_joint(_alloc)
    , pos(_alloc)
    , ori(_alloc)  {
  (void)_alloc;
    }



   typedef std::vector<float, typename ContainerAllocator::template rebind<float>::other >  _ref_joint_type;
  _ref_joint_type ref_joint;

   typedef std::vector<float, typename ContainerAllocator::template rebind<float>::other >  _pos_type;
  _pos_type pos;

   typedef std::vector<float, typename ContainerAllocator::template rebind<float>::other >  _ori_type;
  _ori_type ori;





  typedef boost::shared_ptr< ::aubo_msgs::GetIKRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::aubo_msgs::GetIKRequest_<ContainerAllocator> const> ConstPtr;

}; // struct GetIKRequest_

typedef ::aubo_msgs::GetIKRequest_<std::allocator<void> > GetIKRequest;

typedef boost::shared_ptr< ::aubo_msgs::GetIKRequest > GetIKRequestPtr;
typedef boost::shared_ptr< ::aubo_msgs::GetIKRequest const> GetIKRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::aubo_msgs::GetIKRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::aubo_msgs::GetIKRequest_<ContainerAllocator> >::stream(s, "", v);
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
struct IsFixedSize< ::aubo_msgs::GetIKRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::aubo_msgs::GetIKRequest_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::aubo_msgs::GetIKRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::aubo_msgs::GetIKRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::aubo_msgs::GetIKRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::aubo_msgs::GetIKRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::aubo_msgs::GetIKRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "bdded2a3ac2ee87cb2790fe996ec5a30";
  }

  static const char* value(const ::aubo_msgs::GetIKRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xbdded2a3ac2ee87cULL;
  static const uint64_t static_value2 = 0xb2790fe996ec5a30ULL;
};

template<class ContainerAllocator>
struct DataType< ::aubo_msgs::GetIKRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "aubo_msgs/GetIKRequest";
  }

  static const char* value(const ::aubo_msgs::GetIKRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::aubo_msgs::GetIKRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float32[] ref_joint\n\
float32[] pos\n\
float32[] ori\n\
";
  }

  static const char* value(const ::aubo_msgs::GetIKRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::aubo_msgs::GetIKRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.ref_joint);
      stream.next(m.pos);
      stream.next(m.ori);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct GetIKRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::aubo_msgs::GetIKRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::aubo_msgs::GetIKRequest_<ContainerAllocator>& v)
  {
    s << indent << "ref_joint[]" << std::endl;
    for (size_t i = 0; i < v.ref_joint.size(); ++i)
    {
      s << indent << "  ref_joint[" << i << "]: ";
      Printer<float>::stream(s, indent + "  ", v.ref_joint[i]);
    }
    s << indent << "pos[]" << std::endl;
    for (size_t i = 0; i < v.pos.size(); ++i)
    {
      s << indent << "  pos[" << i << "]: ";
      Printer<float>::stream(s, indent + "  ", v.pos[i]);
    }
    s << indent << "ori[]" << std::endl;
    for (size_t i = 0; i < v.ori.size(); ++i)
    {
      s << indent << "  ori[" << i << "]: ";
      Printer<float>::stream(s, indent + "  ", v.ori[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // AUBO_MSGS_MESSAGE_GETIKREQUEST_H
