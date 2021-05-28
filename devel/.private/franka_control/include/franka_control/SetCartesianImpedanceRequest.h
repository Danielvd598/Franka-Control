// Generated by gencpp from file franka_control/SetCartesianImpedanceRequest.msg
// DO NOT EDIT!


#ifndef FRANKA_CONTROL_MESSAGE_SETCARTESIANIMPEDANCEREQUEST_H
#define FRANKA_CONTROL_MESSAGE_SETCARTESIANIMPEDANCEREQUEST_H


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
struct SetCartesianImpedanceRequest_
{
  typedef SetCartesianImpedanceRequest_<ContainerAllocator> Type;

  SetCartesianImpedanceRequest_()
    : cartesian_stiffness()  {
      cartesian_stiffness.assign(0.0);
  }
  SetCartesianImpedanceRequest_(const ContainerAllocator& _alloc)
    : cartesian_stiffness()  {
  (void)_alloc;
      cartesian_stiffness.assign(0.0);
  }



   typedef boost::array<double, 6>  _cartesian_stiffness_type;
  _cartesian_stiffness_type cartesian_stiffness;





  typedef boost::shared_ptr< ::franka_control::SetCartesianImpedanceRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::franka_control::SetCartesianImpedanceRequest_<ContainerAllocator> const> ConstPtr;

}; // struct SetCartesianImpedanceRequest_

typedef ::franka_control::SetCartesianImpedanceRequest_<std::allocator<void> > SetCartesianImpedanceRequest;

typedef boost::shared_ptr< ::franka_control::SetCartesianImpedanceRequest > SetCartesianImpedanceRequestPtr;
typedef boost::shared_ptr< ::franka_control::SetCartesianImpedanceRequest const> SetCartesianImpedanceRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::franka_control::SetCartesianImpedanceRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::franka_control::SetCartesianImpedanceRequest_<ContainerAllocator> >::stream(s, "", v);
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
struct IsFixedSize< ::franka_control::SetCartesianImpedanceRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::franka_control::SetCartesianImpedanceRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::franka_control::SetCartesianImpedanceRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::franka_control::SetCartesianImpedanceRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::franka_control::SetCartesianImpedanceRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::franka_control::SetCartesianImpedanceRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::franka_control::SetCartesianImpedanceRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "591a43081c539ee56ec83a33587e68c4";
  }

  static const char* value(const ::franka_control::SetCartesianImpedanceRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x591a43081c539ee5ULL;
  static const uint64_t static_value2 = 0x6ec83a33587e68c4ULL;
};

template<class ContainerAllocator>
struct DataType< ::franka_control::SetCartesianImpedanceRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "franka_control/SetCartesianImpedanceRequest";
  }

  static const char* value(const ::franka_control::SetCartesianImpedanceRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::franka_control::SetCartesianImpedanceRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float64[6] cartesian_stiffness\n\
";
  }

  static const char* value(const ::franka_control::SetCartesianImpedanceRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::franka_control::SetCartesianImpedanceRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.cartesian_stiffness);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct SetCartesianImpedanceRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::franka_control::SetCartesianImpedanceRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::franka_control::SetCartesianImpedanceRequest_<ContainerAllocator>& v)
  {
    s << indent << "cartesian_stiffness[]" << std::endl;
    for (size_t i = 0; i < v.cartesian_stiffness.size(); ++i)
    {
      s << indent << "  cartesian_stiffness[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.cartesian_stiffness[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // FRANKA_CONTROL_MESSAGE_SETCARTESIANIMPEDANCEREQUEST_H
