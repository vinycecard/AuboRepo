// Generated by gencpp from file aubo_msgs/GoalPoint.msg
// DO NOT EDIT!


#ifndef AUBO_MSGS_MESSAGE_GOALPOINT_H
#define AUBO_MSGS_MESSAGE_GOALPOINT_H


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
struct GoalPoint_
{
  typedef GoalPoint_<ContainerAllocator> Type;

  GoalPoint_()
    : bus(0)
    , joint1(0.0)
    , joint2(0.0)
    , joint3(0.0)
    , joint4(0.0)
    , joint5(0.0)
    , joint6(0.0)  {
    }
  GoalPoint_(const ContainerAllocator& _alloc)
    : bus(0)
    , joint1(0.0)
    , joint2(0.0)
    , joint3(0.0)
    , joint4(0.0)
    , joint5(0.0)
    , joint6(0.0)  {
  (void)_alloc;
    }



   typedef int8_t _bus_type;
  _bus_type bus;

   typedef float _joint1_type;
  _joint1_type joint1;

   typedef float _joint2_type;
  _joint2_type joint2;

   typedef float _joint3_type;
  _joint3_type joint3;

   typedef float _joint4_type;
  _joint4_type joint4;

   typedef float _joint5_type;
  _joint5_type joint5;

   typedef float _joint6_type;
  _joint6_type joint6;





  typedef boost::shared_ptr< ::aubo_msgs::GoalPoint_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::aubo_msgs::GoalPoint_<ContainerAllocator> const> ConstPtr;

}; // struct GoalPoint_

typedef ::aubo_msgs::GoalPoint_<std::allocator<void> > GoalPoint;

typedef boost::shared_ptr< ::aubo_msgs::GoalPoint > GoalPointPtr;
typedef boost::shared_ptr< ::aubo_msgs::GoalPoint const> GoalPointConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::aubo_msgs::GoalPoint_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::aubo_msgs::GoalPoint_<ContainerAllocator> >::stream(s, "", v);
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
struct IsFixedSize< ::aubo_msgs::GoalPoint_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::aubo_msgs::GoalPoint_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::aubo_msgs::GoalPoint_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::aubo_msgs::GoalPoint_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::aubo_msgs::GoalPoint_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::aubo_msgs::GoalPoint_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::aubo_msgs::GoalPoint_<ContainerAllocator> >
{
  static const char* value()
  {
    return "b4f50b6b5ea74aab23aa569e10ad3edc";
  }

  static const char* value(const ::aubo_msgs::GoalPoint_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xb4f50b6b5ea74aabULL;
  static const uint64_t static_value2 = 0x23aa569e10ad3edcULL;
};

template<class ContainerAllocator>
struct DataType< ::aubo_msgs::GoalPoint_<ContainerAllocator> >
{
  static const char* value()
  {
    return "aubo_msgs/GoalPoint";
  }

  static const char* value(const ::aubo_msgs::GoalPoint_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::aubo_msgs::GoalPoint_<ContainerAllocator> >
{
  static const char* value()
  {
    return "int8 bus\n\
float32 joint1\n\
float32 joint2\n\
float32 joint3\n\
float32 joint4\n\
float32 joint5\n\
float32 joint6\n\
";
  }

  static const char* value(const ::aubo_msgs::GoalPoint_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::aubo_msgs::GoalPoint_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.bus);
      stream.next(m.joint1);
      stream.next(m.joint2);
      stream.next(m.joint3);
      stream.next(m.joint4);
      stream.next(m.joint5);
      stream.next(m.joint6);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct GoalPoint_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::aubo_msgs::GoalPoint_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::aubo_msgs::GoalPoint_<ContainerAllocator>& v)
  {
    s << indent << "bus: ";
    Printer<int8_t>::stream(s, indent + "  ", v.bus);
    s << indent << "joint1: ";
    Printer<float>::stream(s, indent + "  ", v.joint1);
    s << indent << "joint2: ";
    Printer<float>::stream(s, indent + "  ", v.joint2);
    s << indent << "joint3: ";
    Printer<float>::stream(s, indent + "  ", v.joint3);
    s << indent << "joint4: ";
    Printer<float>::stream(s, indent + "  ", v.joint4);
    s << indent << "joint5: ";
    Printer<float>::stream(s, indent + "  ", v.joint5);
    s << indent << "joint6: ";
    Printer<float>::stream(s, indent + "  ", v.joint6);
  }
};

} // namespace message_operations
} // namespace ros

#endif // AUBO_MSGS_MESSAGE_GOALPOINT_H
