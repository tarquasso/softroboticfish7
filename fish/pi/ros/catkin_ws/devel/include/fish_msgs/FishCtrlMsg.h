// Generated by gencpp from file fish_msgs/FishCtrlMsg.msg
// DO NOT EDIT!


#ifndef FISH_MSGS_MESSAGE_FISHCTRLMSG_H
#define FISH_MSGS_MESSAGE_FISHCTRLMSG_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace fish_msgs
{
template <class ContainerAllocator>
struct FishCtrlMsg_
{
  typedef FishCtrlMsg_<ContainerAllocator> Type;

  FishCtrlMsg_()
    : mode(0)
    , thrust(0.0)
    , dvalue(0.0)
    , yaw(0.0)
    , valve(0.0)  {
    }
  FishCtrlMsg_(const ContainerAllocator& _alloc)
    : mode(0)
    , thrust(0.0)
    , dvalue(0.0)
    , yaw(0.0)
    , valve(0.0)  {
  (void)_alloc;
    }



   typedef int8_t _mode_type;
  _mode_type mode;

   typedef float _thrust_type;
  _thrust_type thrust;

   typedef float _dvalue_type;
  _dvalue_type dvalue;

   typedef float _yaw_type;
  _yaw_type yaw;

   typedef float _valve_type;
  _valve_type valve;




  typedef boost::shared_ptr< ::fish_msgs::FishCtrlMsg_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::fish_msgs::FishCtrlMsg_<ContainerAllocator> const> ConstPtr;

}; // struct FishCtrlMsg_

typedef ::fish_msgs::FishCtrlMsg_<std::allocator<void> > FishCtrlMsg;

typedef boost::shared_ptr< ::fish_msgs::FishCtrlMsg > FishCtrlMsgPtr;
typedef boost::shared_ptr< ::fish_msgs::FishCtrlMsg const> FishCtrlMsgConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::fish_msgs::FishCtrlMsg_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::fish_msgs::FishCtrlMsg_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace fish_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'fish_msgs': ['/home/fish/softroboticfish6/fish/pi/ros/catkin_ws/src/fish_msgs/msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::fish_msgs::FishCtrlMsg_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::fish_msgs::FishCtrlMsg_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::fish_msgs::FishCtrlMsg_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::fish_msgs::FishCtrlMsg_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::fish_msgs::FishCtrlMsg_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::fish_msgs::FishCtrlMsg_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::fish_msgs::FishCtrlMsg_<ContainerAllocator> >
{
  static const char* value()
  {
    return "9305e87d9033eb2ef9f1c3927209a31b";
  }

  static const char* value(const ::fish_msgs::FishCtrlMsg_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x9305e87d9033eb2eULL;
  static const uint64_t static_value2 = 0xf9f1c3927209a31bULL;
};

template<class ContainerAllocator>
struct DataType< ::fish_msgs::FishCtrlMsg_<ContainerAllocator> >
{
  static const char* value()
  {
    return "fish_msgs/FishCtrlMsg";
  }

  static const char* value(const ::fish_msgs::FishCtrlMsg_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::fish_msgs::FishCtrlMsg_<ContainerAllocator> >
{
  static const char* value()
  {
    return "int8 mode\n\
float32 thrust\n\
float32 dvalue\n\
float32 yaw\n\
float32 valve\n\
";
  }

  static const char* value(const ::fish_msgs::FishCtrlMsg_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::fish_msgs::FishCtrlMsg_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.mode);
      stream.next(m.thrust);
      stream.next(m.dvalue);
      stream.next(m.yaw);
      stream.next(m.valve);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER;
  }; // struct FishCtrlMsg_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::fish_msgs::FishCtrlMsg_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::fish_msgs::FishCtrlMsg_<ContainerAllocator>& v)
  {
    s << indent << "mode: ";
    Printer<int8_t>::stream(s, indent + "  ", v.mode);
    s << indent << "thrust: ";
    Printer<float>::stream(s, indent + "  ", v.thrust);
    s << indent << "dvalue: ";
    Printer<float>::stream(s, indent + "  ", v.dvalue);
    s << indent << "yaw: ";
    Printer<float>::stream(s, indent + "  ", v.yaw);
    s << indent << "valve: ";
    Printer<float>::stream(s, indent + "  ", v.valve);
  }
};

} // namespace message_operations
} // namespace ros

#endif // FISH_MSGS_MESSAGE_FISHCTRLMSG_H