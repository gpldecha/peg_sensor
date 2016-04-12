// Generated by gencpp from file peg_sensor/String_cmdRequest.msg
// DO NOT EDIT!


#ifndef PEG_SENSOR_MESSAGE_STRING_CMDREQUEST_H
#define PEG_SENSOR_MESSAGE_STRING_CMDREQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace peg_sensor
{
template <class ContainerAllocator>
struct String_cmdRequest_
{
  typedef String_cmdRequest_<ContainerAllocator> Type;

  String_cmdRequest_()
    : req()  {
    }
  String_cmdRequest_(const ContainerAllocator& _alloc)
    : req(_alloc)  {
    }



   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _req_type;
  _req_type req;




  typedef boost::shared_ptr< ::peg_sensor::String_cmdRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::peg_sensor::String_cmdRequest_<ContainerAllocator> const> ConstPtr;

}; // struct String_cmdRequest_

typedef ::peg_sensor::String_cmdRequest_<std::allocator<void> > String_cmdRequest;

typedef boost::shared_ptr< ::peg_sensor::String_cmdRequest > String_cmdRequestPtr;
typedef boost::shared_ptr< ::peg_sensor::String_cmdRequest const> String_cmdRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::peg_sensor::String_cmdRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::peg_sensor::String_cmdRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace peg_sensor

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/opt/ros/indigo/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::peg_sensor::String_cmdRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::peg_sensor::String_cmdRequest_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::peg_sensor::String_cmdRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::peg_sensor::String_cmdRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::peg_sensor::String_cmdRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::peg_sensor::String_cmdRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::peg_sensor::String_cmdRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "b8dc53fbc3707f169aa5dbf7b19d2567";
  }

  static const char* value(const ::peg_sensor::String_cmdRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xb8dc53fbc3707f16ULL;
  static const uint64_t static_value2 = 0x9aa5dbf7b19d2567ULL;
};

template<class ContainerAllocator>
struct DataType< ::peg_sensor::String_cmdRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "peg_sensor/String_cmdRequest";
  }

  static const char* value(const ::peg_sensor::String_cmdRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::peg_sensor::String_cmdRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "string req\n\
";
  }

  static const char* value(const ::peg_sensor::String_cmdRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::peg_sensor::String_cmdRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.req);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER;
  }; // struct String_cmdRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::peg_sensor::String_cmdRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::peg_sensor::String_cmdRequest_<ContainerAllocator>& v)
  {
    s << indent << "req: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.req);
  }
};

} // namespace message_operations
} // namespace ros

#endif // PEG_SENSOR_MESSAGE_STRING_CMDREQUEST_H
