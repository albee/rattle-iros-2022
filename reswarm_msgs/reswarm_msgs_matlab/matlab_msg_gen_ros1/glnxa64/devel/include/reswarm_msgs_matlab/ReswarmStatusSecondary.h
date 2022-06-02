// Generated by gencpp from file reswarm_msgs_matlab/ReswarmStatusSecondary.msg
// DO NOT EDIT!


#ifndef RESWARM_MSGS_MATLAB_MESSAGE_RESWARMSTATUSSECONDARY_H
#define RESWARM_MSGS_MATLAB_MESSAGE_RESWARMSTATUSSECONDARY_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace reswarm_msgs_matlab
{
template <class ContainerAllocator>
struct ReswarmStatusSecondary_
{
  typedef ReswarmStatusSecondary_<ContainerAllocator> Type;

  ReswarmStatusSecondary_()
    : stamp()
    , test_number(0)
    , default_control(false)
    , flight_mode()
    , test_finished(false)
    , coord_ok(false)
    , solver_status(0)
    , cost_value(0.0)
    , kkt_value(0.0)
    , sol_time(0.0)  {
    }
  ReswarmStatusSecondary_(const ContainerAllocator& _alloc)
    : stamp()
    , test_number(0)
    , default_control(false)
    , flight_mode(_alloc)
    , test_finished(false)
    , coord_ok(false)
    , solver_status(0)
    , cost_value(0.0)
    , kkt_value(0.0)
    , sol_time(0.0)  {
  (void)_alloc;
    }



   typedef ros::Time _stamp_type;
  _stamp_type stamp;

   typedef int32_t _test_number_type;
  _test_number_type test_number;

   typedef uint8_t _default_control_type;
  _default_control_type default_control;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _flight_mode_type;
  _flight_mode_type flight_mode;

   typedef uint8_t _test_finished_type;
  _test_finished_type test_finished;

   typedef uint8_t _coord_ok_type;
  _coord_ok_type coord_ok;

   typedef int32_t _solver_status_type;
  _solver_status_type solver_status;

   typedef float _cost_value_type;
  _cost_value_type cost_value;

   typedef float _kkt_value_type;
  _kkt_value_type kkt_value;

   typedef float _sol_time_type;
  _sol_time_type sol_time;





  typedef boost::shared_ptr< ::reswarm_msgs_matlab::ReswarmStatusSecondary_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::reswarm_msgs_matlab::ReswarmStatusSecondary_<ContainerAllocator> const> ConstPtr;

}; // struct ReswarmStatusSecondary_

typedef ::reswarm_msgs_matlab::ReswarmStatusSecondary_<std::allocator<void> > ReswarmStatusSecondary;

typedef boost::shared_ptr< ::reswarm_msgs_matlab::ReswarmStatusSecondary > ReswarmStatusSecondaryPtr;
typedef boost::shared_ptr< ::reswarm_msgs_matlab::ReswarmStatusSecondary const> ReswarmStatusSecondaryConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::reswarm_msgs_matlab::ReswarmStatusSecondary_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::reswarm_msgs_matlab::ReswarmStatusSecondary_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace reswarm_msgs_matlab

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'geometry_msgs': ['/usr/local/MATLAB/R2021a/sys/ros1/glnxa64/ros1/share/geometry_msgs/cmake/../msg'], 'std_msgs': ['/usr/local/MATLAB/R2021a/sys/ros1/glnxa64/ros1/share/std_msgs/cmake/../msg', '/usr/local/MATLAB/R2021a/sys/ros1/glnxa64/ros1/share/std_msgs/cmake/../msg'], 'reswarm_msgs_matlab': ['/home/albee/workspaces/astrobee-ws-reswarm/freeflyer-reswarm/develop/reswarm_msgs/reswarm_msgs_matlab/matlab_msg_gen_ros1/glnxa64/src/reswarm_msgs_matlab/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::reswarm_msgs_matlab::ReswarmStatusSecondary_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::reswarm_msgs_matlab::ReswarmStatusSecondary_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::reswarm_msgs_matlab::ReswarmStatusSecondary_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::reswarm_msgs_matlab::ReswarmStatusSecondary_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::reswarm_msgs_matlab::ReswarmStatusSecondary_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::reswarm_msgs_matlab::ReswarmStatusSecondary_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::reswarm_msgs_matlab::ReswarmStatusSecondary_<ContainerAllocator> >
{
  static const char* value()
  {
    return "758d13e0328708709b74b234c19edbd1";
  }

  static const char* value(const ::reswarm_msgs_matlab::ReswarmStatusSecondary_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x758d13e032870870ULL;
  static const uint64_t static_value2 = 0x9b74b234c19edbd1ULL;
};

template<class ContainerAllocator>
struct DataType< ::reswarm_msgs_matlab::ReswarmStatusSecondary_<ContainerAllocator> >
{
  static const char* value()
  {
    return "reswarm_msgs_matlab/ReswarmStatusSecondary";
  }

  static const char* value(const ::reswarm_msgs_matlab::ReswarmStatusSecondary_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::reswarm_msgs_matlab::ReswarmStatusSecondary_<ContainerAllocator> >
{
  static const char* value()
  {
    return "time stamp\n"
"\n"
"# Config Info\n"
"# base (shared) values\n"
"int32 test_number\n"
"bool default_control\n"
"string flight_mode\n"
"bool test_finished\n"
"bool coord_ok\n"
"\n"
"# SecondaryStatus\n"
"# DMPC Status Flags\n"
"int32 solver_status\n"
"float32 cost_value\n"
"float32 kkt_value\n"
"float32 sol_time\n"
;
  }

  static const char* value(const ::reswarm_msgs_matlab::ReswarmStatusSecondary_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::reswarm_msgs_matlab::ReswarmStatusSecondary_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.stamp);
      stream.next(m.test_number);
      stream.next(m.default_control);
      stream.next(m.flight_mode);
      stream.next(m.test_finished);
      stream.next(m.coord_ok);
      stream.next(m.solver_status);
      stream.next(m.cost_value);
      stream.next(m.kkt_value);
      stream.next(m.sol_time);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct ReswarmStatusSecondary_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::reswarm_msgs_matlab::ReswarmStatusSecondary_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::reswarm_msgs_matlab::ReswarmStatusSecondary_<ContainerAllocator>& v)
  {
    s << indent << "stamp: ";
    Printer<ros::Time>::stream(s, indent + "  ", v.stamp);
    s << indent << "test_number: ";
    Printer<int32_t>::stream(s, indent + "  ", v.test_number);
    s << indent << "default_control: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.default_control);
    s << indent << "flight_mode: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.flight_mode);
    s << indent << "test_finished: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.test_finished);
    s << indent << "coord_ok: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.coord_ok);
    s << indent << "solver_status: ";
    Printer<int32_t>::stream(s, indent + "  ", v.solver_status);
    s << indent << "cost_value: ";
    Printer<float>::stream(s, indent + "  ", v.cost_value);
    s << indent << "kkt_value: ";
    Printer<float>::stream(s, indent + "  ", v.kkt_value);
    s << indent << "sol_time: ";
    Printer<float>::stream(s, indent + "  ", v.sol_time);
  }
};

} // namespace message_operations
} // namespace ros

#endif // RESWARM_MSGS_MATLAB_MESSAGE_RESWARMSTATUSSECONDARY_H
