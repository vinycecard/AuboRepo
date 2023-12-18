// Generated by gencpp from file aubo_msgs/SetIO.msg
// DO NOT EDIT!


#ifndef AUBO_MSGS_MESSAGE_SETIO_H
#define AUBO_MSGS_MESSAGE_SETIO_H

#include <ros/service_traits.h>


#include <aubo_msgs/SetIORequest.h>
#include <aubo_msgs/SetIOResponse.h>


namespace aubo_msgs
{

struct SetIO
{

typedef SetIORequest Request;
typedef SetIOResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct SetIO
} // namespace aubo_msgs


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::aubo_msgs::SetIO > {
  static const char* value()
  {
    return "344aecce520e21c4db398a398eb12110";
  }

  static const char* value(const ::aubo_msgs::SetIO&) { return value(); }
};

template<>
struct DataType< ::aubo_msgs::SetIO > {
  static const char* value()
  {
    return "aubo_msgs/SetIO";
  }

  static const char* value(const ::aubo_msgs::SetIO&) { return value(); }
};


// service_traits::MD5Sum< ::aubo_msgs::SetIORequest> should match 
// service_traits::MD5Sum< ::aubo_msgs::SetIO > 
template<>
struct MD5Sum< ::aubo_msgs::SetIORequest>
{
  static const char* value()
  {
    return MD5Sum< ::aubo_msgs::SetIO >::value();
  }
  static const char* value(const ::aubo_msgs::SetIORequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::aubo_msgs::SetIORequest> should match 
// service_traits::DataType< ::aubo_msgs::SetIO > 
template<>
struct DataType< ::aubo_msgs::SetIORequest>
{
  static const char* value()
  {
    return DataType< ::aubo_msgs::SetIO >::value();
  }
  static const char* value(const ::aubo_msgs::SetIORequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::aubo_msgs::SetIOResponse> should match 
// service_traits::MD5Sum< ::aubo_msgs::SetIO > 
template<>
struct MD5Sum< ::aubo_msgs::SetIOResponse>
{
  static const char* value()
  {
    return MD5Sum< ::aubo_msgs::SetIO >::value();
  }
  static const char* value(const ::aubo_msgs::SetIOResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::aubo_msgs::SetIOResponse> should match 
// service_traits::DataType< ::aubo_msgs::SetIO > 
template<>
struct DataType< ::aubo_msgs::SetIOResponse>
{
  static const char* value()
  {
    return DataType< ::aubo_msgs::SetIO >::value();
  }
  static const char* value(const ::aubo_msgs::SetIOResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // AUBO_MSGS_MESSAGE_SETIO_H