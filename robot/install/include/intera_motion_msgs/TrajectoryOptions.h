// Generated by gencpp from file intera_motion_msgs/TrajectoryOptions.msg
// DO NOT EDIT!


#ifndef INTERA_MOTION_MSGS_MESSAGE_TRAJECTORYOPTIONS_H
#define INTERA_MOTION_MSGS_MESSAGE_TRAJECTORYOPTIONS_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <intera_core_msgs/InteractionControlCommand.h>
#include <intera_motion_msgs/TrackingOptions.h>

namespace intera_motion_msgs
{
template <class ContainerAllocator>
struct TrajectoryOptions_
{
  typedef TrajectoryOptions_<ContainerAllocator> Type;

  TrajectoryOptions_()
    : interpolation_type()
    , interaction_control(false)
    , interaction_params()
    , nso_start_offset_allowed(false)
    , nso_check_end_offset(false)
    , tracking_options()
    , end_time()
    , path_interpolation_step(0.0)  {
    }
  TrajectoryOptions_(const ContainerAllocator& _alloc)
    : interpolation_type(_alloc)
    , interaction_control(false)
    , interaction_params(_alloc)
    , nso_start_offset_allowed(false)
    , nso_check_end_offset(false)
    , tracking_options(_alloc)
    , end_time()
    , path_interpolation_step(0.0)  {
  (void)_alloc;
    }



   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _interpolation_type_type;
  _interpolation_type_type interpolation_type;

   typedef uint8_t _interaction_control_type;
  _interaction_control_type interaction_control;

   typedef  ::intera_core_msgs::InteractionControlCommand_<ContainerAllocator>  _interaction_params_type;
  _interaction_params_type interaction_params;

   typedef uint8_t _nso_start_offset_allowed_type;
  _nso_start_offset_allowed_type nso_start_offset_allowed;

   typedef uint8_t _nso_check_end_offset_type;
  _nso_check_end_offset_type nso_check_end_offset;

   typedef  ::intera_motion_msgs::TrackingOptions_<ContainerAllocator>  _tracking_options_type;
  _tracking_options_type tracking_options;

   typedef ros::Time _end_time_type;
  _end_time_type end_time;

   typedef double _path_interpolation_step_type;
  _path_interpolation_step_type path_interpolation_step;




  static const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  CARTESIAN;
  static const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  JOINT;

  typedef boost::shared_ptr< ::intera_motion_msgs::TrajectoryOptions_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::intera_motion_msgs::TrajectoryOptions_<ContainerAllocator> const> ConstPtr;

}; // struct TrajectoryOptions_

typedef ::intera_motion_msgs::TrajectoryOptions_<std::allocator<void> > TrajectoryOptions;

typedef boost::shared_ptr< ::intera_motion_msgs::TrajectoryOptions > TrajectoryOptionsPtr;
typedef boost::shared_ptr< ::intera_motion_msgs::TrajectoryOptions const> TrajectoryOptionsConstPtr;

// constants requiring out of line definition

   
   template<typename ContainerAllocator> const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > 
      TrajectoryOptions_<ContainerAllocator>::CARTESIAN =
        
          "CARTESIAN"
        
        ;
   

   
   template<typename ContainerAllocator> const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > 
      TrajectoryOptions_<ContainerAllocator>::JOINT =
        
          "JOINT"
        
        ;
   



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::intera_motion_msgs::TrajectoryOptions_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::intera_motion_msgs::TrajectoryOptions_<ContainerAllocator> >::stream(s, "", v);
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
struct IsFixedSize< ::intera_motion_msgs::TrajectoryOptions_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::intera_motion_msgs::TrajectoryOptions_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::intera_motion_msgs::TrajectoryOptions_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::intera_motion_msgs::TrajectoryOptions_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::intera_motion_msgs::TrajectoryOptions_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::intera_motion_msgs::TrajectoryOptions_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::intera_motion_msgs::TrajectoryOptions_<ContainerAllocator> >
{
  static const char* value()
  {
    return "d6c6806743ac9695334265046d57235e";
  }

  static const char* value(const ::intera_motion_msgs::TrajectoryOptions_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xd6c6806743ac9695ULL;
  static const uint64_t static_value2 = 0x334265046d57235eULL;
};

template<class ContainerAllocator>
struct DataType< ::intera_motion_msgs::TrajectoryOptions_<ContainerAllocator> >
{
  static const char* value()
  {
    return "intera_motion_msgs/TrajectoryOptions";
  }

  static const char* value(const ::intera_motion_msgs::TrajectoryOptions_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::intera_motion_msgs::TrajectoryOptions_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# Trajectory interpolation type\n\
string CARTESIAN=CARTESIAN\n\
string JOINT=JOINT\n\
string interpolation_type\n\
\n\
# True if the trajectory uses interaction control, false for position control.\n\
bool interaction_control\n\
\n\
# Interaction control parameters\n\
intera_core_msgs/InteractionControlCommand interaction_params\n\
\n\
# Allow small joint adjustments at the beginning of Cartesian trajectories.\n\
# Set to false for 'small' motions.\n\
bool nso_start_offset_allowed\n\
\n\
# Check the offset at the end of a Cartesian trajectory from the final waypoint nullspace goal.\n\
bool nso_check_end_offset\n\
\n\
# Options for the tracking controller:\n\
TrackingOptions tracking_options\n\
\n\
# Desired trajectory end time, ROS timestamp\n\
time end_time\n\
\n\
# The rate in seconds that the path is interpolated and returned back to the user\n\
# No interpolation will happen if set to zero\n\
float64 path_interpolation_step\n\
\n\
================================================================================\n\
MSG: intera_core_msgs/InteractionControlCommand\n\
# Message sets the interaction (impedance/force) control on or off\n\
# It also contains desired cartesian stiffness K, damping D, and force values\n\
\n\
Header header\n\
bool      interaction_control_active\n\
\n\
## Cartesian Impedance Control Parameters\n\
# Stiffness units are (N/m) for first 3 and (Nm/rad) for second 3 values\n\
float64[] K_impedance\n\
# Force certain directions to have maximum possible impedance for a given pose\n\
bool[] max_impedance\n\
# Damping units are (Ns/m) for first 3 and (Nms/rad) for the second 3 values\n\
float64[] D_impedance\n\
# Joint Nullspace stiffness units are in (Nm/rad) (length == number of joints)\n\
float64[] K_nullspace\n\
\n\
## Parameters for force control or impedance control with force limit\n\
# If in force mode, this is the vector of desired forces/torques\n\
# to be regulated in (N) and (Nm)\n\
# If in impedance with force limit mode, this vector specifies the\n\
# magnitude of forces/torques (N and Nm) that the command will not exceed.\n\
float64[] force_command\n\
\n\
## Desired frame\n\
geometry_msgs/Pose interaction_frame\n\
string endpoint_name\n\
# True if impedance and force commands are defined in endpoint frame\n\
bool in_endpoint_frame\n\
\n\
# Set to true to disable damping during force control. Damping is used\n\
# to slow down robot motion during force control in free space.\n\
# Option included for SDK users to disable damping in force control\n\
bool disable_damping_in_force_control\n\
\n\
# Set to true to disable reference resetting. Reference resetting is\n\
# used when interaction parameters change, in order to avoid jumps/jerks.\n\
# Option included for SDK users to disable reference resetting if the\n\
# intention is to change interaction parameters.\n\
bool disable_reference_resetting\n\
\n\
## Mode Selection Parameters\n\
# The possible interaction control modes are:\n\
# Impedance mode: implements desired endpoint stiffness and damping.\n\
uint8 IMPEDANCE_MODE=1\n\
# Force mode: applies force/torque in the specified dimensions.\n\
uint8 FORCE_MODE=2\n\
# Impedance with force limit: impedance control while ensuring the commanded\n\
# forces/torques do not exceed force_command.\n\
uint8 IMPEDANCE_WITH_FORCE_LIMIT_MODE=3\n\
# Force with motion bounds: force control while ensuring the current\n\
# pose/velocities do not exceed forceMotionThreshold (currenetly defined in yaml)\n\
uint8 FORCE_WITH_MOTION_LIMIT_MODE=4\n\
\n\
# Specifies the interaction control mode for each Cartesian dimension (6)\n\
uint8[] interaction_control_mode\n\
\n\
# All 6 values in force and impedance parameter vectors have to be filled,\n\
# If a control mode is not used in a Cartesian dimension,\n\
# the corresponding parameters will be ignored.\n\
\n\
## Parameters for Constrained Zero-G Behaviors\n\
# Allow for arbitrary rotational displacements from the current orientation\n\
# for constrained zero-G. Setting 'rotations_for_constrained_zeroG = True'\n\
# will disable the rotational stiffness field which limits rotational\n\
# displacements to +/- 82.5 degree.\n\
# NOTE: it will be only enabled for a stationary reference orientation\n\
bool rotations_for_constrained_zeroG\n\
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
MSG: geometry_msgs/Pose\n\
# A representation of pose in free space, composed of position and orientation. \n\
Point position\n\
Quaternion orientation\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Point\n\
# This contains the position of a point in free space\n\
float64 x\n\
float64 y\n\
float64 z\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Quaternion\n\
# This represents an orientation in free space in quaternion form.\n\
\n\
float64 x\n\
float64 y\n\
float64 z\n\
float64 w\n\
\n\
================================================================================\n\
MSG: intera_motion_msgs/TrackingOptions\n\
# Minimum trajectory tracking time rate:  (default = less than one)\n\
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

  static const char* value(const ::intera_motion_msgs::TrajectoryOptions_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::intera_motion_msgs::TrajectoryOptions_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.interpolation_type);
      stream.next(m.interaction_control);
      stream.next(m.interaction_params);
      stream.next(m.nso_start_offset_allowed);
      stream.next(m.nso_check_end_offset);
      stream.next(m.tracking_options);
      stream.next(m.end_time);
      stream.next(m.path_interpolation_step);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct TrajectoryOptions_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::intera_motion_msgs::TrajectoryOptions_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::intera_motion_msgs::TrajectoryOptions_<ContainerAllocator>& v)
  {
    s << indent << "interpolation_type: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.interpolation_type);
    s << indent << "interaction_control: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.interaction_control);
    s << indent << "interaction_params: ";
    s << std::endl;
    Printer< ::intera_core_msgs::InteractionControlCommand_<ContainerAllocator> >::stream(s, indent + "  ", v.interaction_params);
    s << indent << "nso_start_offset_allowed: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.nso_start_offset_allowed);
    s << indent << "nso_check_end_offset: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.nso_check_end_offset);
    s << indent << "tracking_options: ";
    s << std::endl;
    Printer< ::intera_motion_msgs::TrackingOptions_<ContainerAllocator> >::stream(s, indent + "  ", v.tracking_options);
    s << indent << "end_time: ";
    Printer<ros::Time>::stream(s, indent + "  ", v.end_time);
    s << indent << "path_interpolation_step: ";
    Printer<double>::stream(s, indent + "  ", v.path_interpolation_step);
  }
};

} // namespace message_operations
} // namespace ros

#endif // INTERA_MOTION_MSGS_MESSAGE_TRAJECTORYOPTIONS_H
