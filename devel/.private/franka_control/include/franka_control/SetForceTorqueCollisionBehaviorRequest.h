// Generated by gencpp from file franka_control/SetForceTorqueCollisionBehaviorRequest.msg
// DO NOT EDIT!


#ifndef FRANKA_CONTROL_MESSAGE_SETFORCETORQUECOLLISIONBEHAVIORREQUEST_H
#define FRANKA_CONTROL_MESSAGE_SETFORCETORQUECOLLISIONBEHAVIORREQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace franka_control
{
template <class ContainerAllocator>
struct SetForceTorqueCollisionBehaviorRequest_
{
  typedef SetForceTorqueCollisionBehaviorRequest_<ContainerAllocator> Type;

  SetForceTorqueCollisionBehaviorRequest_()
    : lower_torque_thresholds_nominal()
    , upper_torque_thresholds_nominal()
    , lower_force_thresholds_nominal()
    , upper_force_thresholds_nominal()  {
      lower_torque_thresholds_nominal.assign(0.0);

      upper_torque_thresholds_nominal.assign(0.0);

      lower_force_thresholds_nominal.assign(0.0);

      upper_force_thresholds_nominal.assign(0.0);
  }
  SetForceTorqueCollisionBehaviorRequest_(const ContainerAllocator& _alloc)
    : lower_torque_thresholds_nominal()
    , upper_torque_thresholds_nominal()
    , lower_force_thresholds_nominal()
    , upper_force_thresholds_nominal()  {
  (void)_alloc;
      lower_torque_thresholds_nominal.assign(0.0);

      upper_torque_thresholds_nominal.assign(0.0);

      lower_force_thresholds_nominal.assign(0.0);

      upper_force_thresholds_nominal.assign(0.0);
  }



   typedef boost::array<double, 7>  _lower_torque_thresholds_nominal_type;
  _lower_torque_thresholds_nominal_type lower_torque_thresholds_nominal;

   typedef boost::array<double, 7>  _upper_torque_thresholds_nominal_type;
  _upper_torque_thresholds_nominal_type upper_torque_thresholds_nominal;

   typedef boost::array<double, 6>  _lower_force_thresholds_nominal_type;
  _lower_force_thresholds_nominal_type lower_force_thresholds_nominal;

   typedef boost::array<double, 6>  _upper_force_thresholds_nominal_type;
  _upper_force_thresholds_nominal_type upper_force_thresholds_nominal;





  typedef boost::shared_ptr< ::franka_control::SetForceTorqueCollisionBehaviorRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::franka_control::SetForceTorqueCollisionBehaviorRequest_<ContainerAllocator> const> ConstPtr;

}; // struct SetForceTorqueCollisionBehaviorRequest_

typedef ::franka_control::SetForceTorqueCollisionBehaviorRequest_<std::allocator<void> > SetForceTorqueCollisionBehaviorRequest;

typedef boost::shared_ptr< ::franka_control::SetForceTorqueCollisionBehaviorRequest > SetForceTorqueCollisionBehaviorRequestPtr;
typedef boost::shared_ptr< ::franka_control::SetForceTorqueCollisionBehaviorRequest const> SetForceTorqueCollisionBehaviorRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::franka_control::SetForceTorqueCollisionBehaviorRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::franka_control::SetForceTorqueCollisionBehaviorRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace franka_control

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'franka_control': ['/home/dijkd/franka_ws/devel/.private/franka_control/share/franka_control/msg'], 'actionlib_msgs': ['/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::franka_control::SetForceTorqueCollisionBehaviorRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::franka_control::SetForceTorqueCollisionBehaviorRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::franka_control::SetForceTorqueCollisionBehaviorRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::franka_control::SetForceTorqueCollisionBehaviorRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::franka_control::SetForceTorqueCollisionBehaviorRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::franka_control::SetForceTorqueCollisionBehaviorRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::franka_control::SetForceTorqueCollisionBehaviorRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "af37de8897f6124b6b82b8dad5d5a876";
  }

  static const char* value(const ::franka_control::SetForceTorqueCollisionBehaviorRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xaf37de8897f6124bULL;
  static const uint64_t static_value2 = 0x6b82b8dad5d5a876ULL;
};

template<class ContainerAllocator>
struct DataType< ::franka_control::SetForceTorqueCollisionBehaviorRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "franka_control/SetForceTorqueCollisionBehaviorRequest";
  }

  static const char* value(const ::franka_control::SetForceTorqueCollisionBehaviorRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::franka_control::SetForceTorqueCollisionBehaviorRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float64[7] lower_torque_thresholds_nominal\n\
float64[7] upper_torque_thresholds_nominal\n\
float64[6] lower_force_thresholds_nominal\n\
float64[6] upper_force_thresholds_nominal\n\
";
  }

  static const char* value(const ::franka_control::SetForceTorqueCollisionBehaviorRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::franka_control::SetForceTorqueCollisionBehaviorRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.lower_torque_thresholds_nominal);
      stream.next(m.upper_torque_thresholds_nominal);
      stream.next(m.lower_force_thresholds_nominal);
      stream.next(m.upper_force_thresholds_nominal);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct SetForceTorqueCollisionBehaviorRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::franka_control::SetForceTorqueCollisionBehaviorRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::franka_control::SetForceTorqueCollisionBehaviorRequest_<ContainerAllocator>& v)
  {
    s << indent << "lower_torque_thresholds_nominal[]" << std::endl;
    for (size_t i = 0; i < v.lower_torque_thresholds_nominal.size(); ++i)
    {
      s << indent << "  lower_torque_thresholds_nominal[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.lower_torque_thresholds_nominal[i]);
    }
    s << indent << "upper_torque_thresholds_nominal[]" << std::endl;
    for (size_t i = 0; i < v.upper_torque_thresholds_nominal.size(); ++i)
    {
      s << indent << "  upper_torque_thresholds_nominal[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.upper_torque_thresholds_nominal[i]);
    }
    s << indent << "lower_force_thresholds_nominal[]" << std::endl;
    for (size_t i = 0; i < v.lower_force_thresholds_nominal.size(); ++i)
    {
      s << indent << "  lower_force_thresholds_nominal[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.lower_force_thresholds_nominal[i]);
    }
    s << indent << "upper_force_thresholds_nominal[]" << std::endl;
    for (size_t i = 0; i < v.upper_force_thresholds_nominal.size(); ++i)
    {
      s << indent << "  upper_force_thresholds_nominal[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.upper_force_thresholds_nominal[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // FRANKA_CONTROL_MESSAGE_SETFORCETORQUECOLLISIONBEHAVIORREQUEST_H
