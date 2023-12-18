// Generated by gencpp from file jog_msgs/JogFrame.msg
// DO NOT EDIT!


#ifndef JOG_MSGS_MESSAGE_JOGFRAME_H
#define JOG_MSGS_MESSAGE_JOGFRAME_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Vector3.h>

namespace jog_msgs
{
template <class ContainerAllocator>
struct JogFrame_
{
  typedef JogFrame_<ContainerAllocator> Type;

  JogFrame_()
    : header()
    , group_name()
    , link_name()
    , linear_delta()
    , angular_delta()
    , avoid_collisions(false)  {
    }
  JogFrame_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , group_name(_alloc)
    , link_name(_alloc)
    , linear_delta(_alloc)
    , angular_delta(_alloc)
    , avoid_collisions(false)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _group_name_type;
  _group_name_type group_name;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _link_name_type;
  _link_name_type link_name;

   typedef  ::geometry_msgs::Vector3_<ContainerAllocator>  _linear_delta_type;
  _linear_delta_type linear_delta;

   typedef  ::geometry_msgs::Vector3_<ContainerAllocator>  _angular_delta_type;
  _angular_delta_type angular_delta;

   typedef uint8_t _avoid_collisions_type;
  _avoid_collisions_type avoid_collisions;





  typedef boost::shared_ptr< ::jog_msgs::JogFrame_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::jog_msgs::JogFrame_<ContainerAllocator> const> ConstPtr;

}; // struct JogFrame_

typedef ::jog_msgs::JogFrame_<std::allocator<void> > JogFrame;

typedef boost::shared_ptr< ::jog_msgs::JogFrame > JogFramePtr;
typedef boost::shared_ptr< ::jog_msgs::JogFrame const> JogFrameConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::jog_msgs::JogFrame_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::jog_msgs::JogFrame_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace jog_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': True}
// {'geometry_msgs': ['/opt/ros/kinetic/share/geometry_msgs/cmake/../msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'jog_msgs': ['/home/nicolas/catkin_ws/src/jog_arm/jog_msgs/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::jog_msgs::JogFrame_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::jog_msgs::JogFrame_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::jog_msgs::JogFrame_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::jog_msgs::JogFrame_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::jog_msgs::JogFrame_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::jog_msgs::JogFrame_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::jog_msgs::JogFrame_<ContainerAllocator> >
{
  static const char* value()
  {
    return "e342f29bf6beaf00261bdae365abfff9";
  }

  static const char* value(const ::jog_msgs::JogFrame_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xe342f29bf6beaf00ULL;
  static const uint64_t static_value2 = 0x261bdae365abfff9ULL;
};

template<class ContainerAllocator>
struct DataType< ::jog_msgs::JogFrame_<ContainerAllocator> >
{
  static const char* value()
  {
    return "jog_msgs/JogFrame";
  }

  static const char* value(const ::jog_msgs::JogFrame_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::jog_msgs::JogFrame_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# This is a message to hold data to jog by specifying a target\n\
# frame. It uses MoveIt! kinematics, so you need to specify the\n\
# JointGroup name to use in group_name. (lienar|angular)_delta is the\n\
# amount of displacement.\n\
\n\
# header message. You must set frame_id to define the reference\n\
# coordinate system of the displacament\n\
Header header\n\
\n\
# Name of JointGroup of MoveIt!\n\
string group_name\n\
\n\
# Target link name to jog. The link must be in the JoingGroup\n\
string link_name\n\
\n\
# Linear displacement vector to jog. The refrence frame is defined by\n\
# frame_id in header. Unit is in meter.\n\
geometry_msgs/Vector3 linear_delta\n\
\n\
# Angular displacement vector to jog. The refrence frame is defined by\n\
# frame_id in header. Unit is in radian.\n\
geometry_msgs/Vector3 angular_delta\n\
\n\
# It uses avoid_collisions option of MoveIt! kinematics. If it is\n\
# true, the robot doesn't move if any collisions occured.\n\
bool avoid_collisions\n\
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

  static const char* value(const ::jog_msgs::JogFrame_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::jog_msgs::JogFrame_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.group_name);
      stream.next(m.link_name);
      stream.next(m.linear_delta);
      stream.next(m.angular_delta);
      stream.next(m.avoid_collisions);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct JogFrame_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::jog_msgs::JogFrame_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::jog_msgs::JogFrame_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "group_name: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.group_name);
    s << indent << "link_name: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.link_name);
    s << indent << "linear_delta: ";
    s << std::endl;
    Printer< ::geometry_msgs::Vector3_<ContainerAllocator> >::stream(s, indent + "  ", v.linear_delta);
    s << indent << "angular_delta: ";
    s << std::endl;
    Printer< ::geometry_msgs::Vector3_<ContainerAllocator> >::stream(s, indent + "  ", v.angular_delta);
    s << indent << "avoid_collisions: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.avoid_collisions);
  }
};

} // namespace message_operations
} // namespace ros

#endif // JOG_MSGS_MESSAGE_JOGFRAME_H