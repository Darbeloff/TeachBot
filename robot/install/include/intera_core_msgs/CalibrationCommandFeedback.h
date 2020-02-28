// Generated by gencpp from file intera_core_msgs/CalibrationCommandFeedback.msg
// DO NOT EDIT!


#ifndef INTERA_CORE_MSGS_MESSAGE_CALIBRATIONCOMMANDFEEDBACK_H
#define INTERA_CORE_MSGS_MESSAGE_CALIBRATIONCOMMANDFEEDBACK_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace intera_core_msgs
{
template <class ContainerAllocator>
struct CalibrationCommandFeedback_
{
  typedef CalibrationCommandFeedback_<ContainerAllocator> Type;

  CalibrationCommandFeedback_()
    : currentState()
    , numberOfPoses(0)
    , currentPoseNumber(0)  {
    }
  CalibrationCommandFeedback_(const ContainerAllocator& _alloc)
    : currentState(_alloc)
    , numberOfPoses(0)
    , currentPoseNumber(0)  {
  (void)_alloc;
    }



   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _currentState_type;
  _currentState_type currentState;

   typedef uint32_t _numberOfPoses_type;
  _numberOfPoses_type numberOfPoses;

   typedef uint32_t _currentPoseNumber_type;
  _currentPoseNumber_type currentPoseNumber;





  typedef boost::shared_ptr< ::intera_core_msgs::CalibrationCommandFeedback_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::intera_core_msgs::CalibrationCommandFeedback_<ContainerAllocator> const> ConstPtr;

}; // struct CalibrationCommandFeedback_

typedef ::intera_core_msgs::CalibrationCommandFeedback_<std::allocator<void> > CalibrationCommandFeedback;

typedef boost::shared_ptr< ::intera_core_msgs::CalibrationCommandFeedback > CalibrationCommandFeedbackPtr;
typedef boost::shared_ptr< ::intera_core_msgs::CalibrationCommandFeedback const> CalibrationCommandFeedbackConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::intera_core_msgs::CalibrationCommandFeedback_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::intera_core_msgs::CalibrationCommandFeedback_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace intera_core_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'intera_core_msgs': ['/home/sarah/TeachBot/robot/src/intera_common/intera_core_msgs/msg', '/home/sarah/TeachBot/robot/devel/share/intera_core_msgs/msg'], 'geometry_msgs': ['/opt/ros/kinetic/share/geometry_msgs/cmake/../msg'], 'actionlib_msgs': ['/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'sensor_msgs': ['/opt/ros/kinetic/share/sensor_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::intera_core_msgs::CalibrationCommandFeedback_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::intera_core_msgs::CalibrationCommandFeedback_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::intera_core_msgs::CalibrationCommandFeedback_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::intera_core_msgs::CalibrationCommandFeedback_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::intera_core_msgs::CalibrationCommandFeedback_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::intera_core_msgs::CalibrationCommandFeedback_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::intera_core_msgs::CalibrationCommandFeedback_<ContainerAllocator> >
{
  static const char* value()
  {
    return "a4e5158c157d9fb03da0faf44b425ee1";
  }

  static const char* value(const ::intera_core_msgs::CalibrationCommandFeedback_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xa4e5158c157d9fb0ULL;
  static const uint64_t static_value2 = 0x3da0faf44b425ee1ULL;
};

template<class ContainerAllocator>
struct DataType< ::intera_core_msgs::CalibrationCommandFeedback_<ContainerAllocator> >
{
  static const char* value()
  {
    return "intera_core_msgs/CalibrationCommandFeedback";
  }

  static const char* value(const ::intera_core_msgs::CalibrationCommandFeedback_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::intera_core_msgs::CalibrationCommandFeedback_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
# feedback\n\
string currentState\n\
uint32 numberOfPoses\n\
uint32 currentPoseNumber\n\
\n\
";
  }

  static const char* value(const ::intera_core_msgs::CalibrationCommandFeedback_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::intera_core_msgs::CalibrationCommandFeedback_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.currentState);
      stream.next(m.numberOfPoses);
      stream.next(m.currentPoseNumber);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct CalibrationCommandFeedback_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::intera_core_msgs::CalibrationCommandFeedback_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::intera_core_msgs::CalibrationCommandFeedback_<ContainerAllocator>& v)
  {
    s << indent << "currentState: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.currentState);
    s << indent << "numberOfPoses: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.numberOfPoses);
    s << indent << "currentPoseNumber: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.currentPoseNumber);
  }
};

} // namespace message_operations
} // namespace ros

#endif // INTERA_CORE_MSGS_MESSAGE_CALIBRATIONCOMMANDFEEDBACK_H
