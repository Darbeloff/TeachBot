// Generated by gencpp from file intera_motion_msgs/TrackingOptions.msg
// DO NOT EDIT!


#ifndef INTERA_MOTION_MSGS_MESSAGE_TRACKINGOPTIONS_H
#define INTERA_MOTION_MSGS_MESSAGE_TRACKINGOPTIONS_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace intera_motion_msgs
{
template <class ContainerAllocator>
struct TrackingOptions_
{
  typedef TrackingOptions_<ContainerAllocator> Type;

  TrackingOptions_()
    : use_min_time_rate(false)
    , min_time_rate(0.0)
    , use_max_time_rate(false)
    , max_time_rate(0.0)
    , goal_joint_tolerance()
    , use_settling_time_at_goal(false)
    , settling_time_at_goal(0.0)  {
    }
  TrackingOptions_(const ContainerAllocator& _alloc)
    : use_min_time_rate(false)
    , min_time_rate(0.0)
    , use_max_time_rate(false)
    , max_time_rate(0.0)
    , goal_joint_tolerance(_alloc)
    , use_settling_time_at_goal(false)
    , settling_time_at_goal(0.0)  {
  (void)_alloc;
    }



   typedef uint8_t _use_min_time_rate_type;
  _use_min_time_rate_type use_min_time_rate;

   typedef double _min_time_rate_type;
  _min_time_rate_type min_time_rate;

   typedef uint8_t _use_max_time_rate_type;
  _use_max_time_rate_type use_max_time_rate;

   typedef double _max_time_rate_type;
  _max_time_rate_type max_time_rate;

   typedef std::vector<double, typename ContainerAllocator::template rebind<double>::other >  _goal_joint_tolerance_type;
  _goal_joint_tolerance_type goal_joint_tolerance;

   typedef uint8_t _use_settling_time_at_goal_type;
  _use_settling_time_at_goal_type use_settling_time_at_goal;

   typedef double _settling_time_at_goal_type;
  _settling_time_at_goal_type settling_time_at_goal;





  typedef boost::shared_ptr< ::intera_motion_msgs::TrackingOptions_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::intera_motion_msgs::TrackingOptions_<ContainerAllocator> const> ConstPtr;

}; // struct TrackingOptions_

typedef ::intera_motion_msgs::TrackingOptions_<std::allocator<void> > TrackingOptions;

typedef boost::shared_ptr< ::intera_motion_msgs::TrackingOptions > TrackingOptionsPtr;
typedef boost::shared_ptr< ::intera_motion_msgs::TrackingOptions const> TrackingOptionsConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::intera_motion_msgs::TrackingOptions_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::intera_motion_msgs::TrackingOptions_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace intera_motion_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'intera_core_msgs': ['/home/sarah/TeachBot/robot/src/intera_common/intera_core_msgs/msg', '/home/sarah/TeachBot/robot/devel/share/intera_core_msgs/msg'], 'sensor_msgs': ['/opt/ros/kinetic/share/sensor_msgs/cmake/../msg'], 'actionlib_msgs': ['/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg'], 'intera_motion_msgs': ['/home/sarah/TeachBot/robot/src/intera_common/intera_motion_msgs/msg', '/home/sarah/TeachBot/robot/devel/share/intera_motion_msgs/msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/kinetic/share/geometry_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::intera_motion_msgs::TrackingOptions_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::intera_motion_msgs::TrackingOptions_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::intera_motion_msgs::TrackingOptions_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::intera_motion_msgs::TrackingOptions_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::intera_motion_msgs::TrackingOptions_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::intera_motion_msgs::TrackingOptions_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::intera_motion_msgs::TrackingOptions_<ContainerAllocator> >
{
  static const char* value()
  {
    return "e848e8a266b514c3bde707d0e1859055";
  }

  static const char* value(const ::intera_motion_msgs::TrackingOptions_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xe848e8a266b514c3ULL;
  static const uint64_t static_value2 = 0xbde707d0e1859055ULL;
};

template<class ContainerAllocator>
struct DataType< ::intera_motion_msgs::TrackingOptions_<ContainerAllocator> >
{
  static const char* value()
  {
    return "intera_motion_msgs/TrackingOptions";
  }

  static const char* value(const ::intera_motion_msgs::TrackingOptions_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::intera_motion_msgs::TrackingOptions_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# Minimum trajectory tracking time rate:  (default = less than one)\n\
bool     use_min_time_rate\n\
float64  min_time_rate\n\
\n\
# Maximum trajectory tracking time rate:  (1.0 = real-time = default)\n\
bool     use_max_time_rate\n\
float64  max_time_rate\n\
\n\
# Angular error tolerance at final point on trajectory (rad)\n\
float64[] goal_joint_tolerance\n\
\n\
# Time for the controller to settle within joint tolerances at the goal (sec)\n\
bool     use_settling_time_at_goal\n\
float64  settling_time_at_goal\n\
";
  }

  static const char* value(const ::intera_motion_msgs::TrackingOptions_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::intera_motion_msgs::TrackingOptions_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.use_min_time_rate);
      stream.next(m.min_time_rate);
      stream.next(m.use_max_time_rate);
      stream.next(m.max_time_rate);
      stream.next(m.goal_joint_tolerance);
      stream.next(m.use_settling_time_at_goal);
      stream.next(m.settling_time_at_goal);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct TrackingOptions_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::intera_motion_msgs::TrackingOptions_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::intera_motion_msgs::TrackingOptions_<ContainerAllocator>& v)
  {
    s << indent << "use_min_time_rate: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.use_min_time_rate);
    s << indent << "min_time_rate: ";
    Printer<double>::stream(s, indent + "  ", v.min_time_rate);
    s << indent << "use_max_time_rate: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.use_max_time_rate);
    s << indent << "max_time_rate: ";
    Printer<double>::stream(s, indent + "  ", v.max_time_rate);
    s << indent << "goal_joint_tolerance[]" << std::endl;
    for (size_t i = 0; i < v.goal_joint_tolerance.size(); ++i)
    {
      s << indent << "  goal_joint_tolerance[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.goal_joint_tolerance[i]);
    }
    s << indent << "use_settling_time_at_goal: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.use_settling_time_at_goal);
    s << indent << "settling_time_at_goal: ";
    Printer<double>::stream(s, indent + "  ", v.settling_time_at_goal);
  }
};

} // namespace message_operations
} // namespace ros

#endif // INTERA_MOTION_MSGS_MESSAGE_TRACKINGOPTIONS_H
