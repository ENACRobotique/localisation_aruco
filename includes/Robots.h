// Generated by gencpp from file cube_pos/Robots.msg
// DO NOT EDIT!


#ifndef CUBE_POS_MESSAGE_ROBOTS_H
#define CUBE_POS_MESSAGE_ROBOTS_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>
#include <Robot.h>

namespace cube_pos
{
    template <class ContainerAllocator>
    struct Robots_
    {
        typedef Robots_<ContainerAllocator> Type;

        Robots_()
                : header()
                , robots()  {
        }
        Robots_(const ContainerAllocator& _alloc)
                : header(_alloc)
                , robots(_alloc)  {
            (void)_alloc;
        }



        typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
        _header_type header;

        typedef std::vector< ::cube_pos::Robot_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::cube_pos::Robot_<ContainerAllocator> >::other >  _robots_type;
        _robots_type robots;




        typedef boost::shared_ptr< ::cube_pos::Robots_<ContainerAllocator> > Ptr;
        typedef boost::shared_ptr< ::cube_pos::Robots_<ContainerAllocator> const> ConstPtr;

    }; // struct Robots_

    typedef ::cube_pos::Robots_<std::allocator<void> > Robots;

    typedef boost::shared_ptr< ::cube_pos::Robots > RobotsPtr;
    typedef boost::shared_ptr< ::cube_pos::Robots const> RobotsConstPtr;

// constants requiring out of line definition



    template<typename ContainerAllocator>
    std::ostream& operator<<(std::ostream& s, const ::cube_pos::Robots_<ContainerAllocator> & v)
    {
        ros::message_operations::Printer< ::cube_pos::Robots_<ContainerAllocator> >::stream(s, "", v);
        return s;
    }

} // namespace cube_pos

namespace ros
{
    namespace message_traits
    {



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': True}
// {'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'cube_pos': ['/home/club_robot/localisation_aruco/msg'], 'geometry_msgs': ['/opt/ros/kinetic/share/geometry_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




        template <class ContainerAllocator>
        struct IsFixedSize< ::cube_pos::Robots_<ContainerAllocator> >
                : FalseType
        { };

        template <class ContainerAllocator>
        struct IsFixedSize< ::cube_pos::Robots_<ContainerAllocator> const>
                : FalseType
        { };

        template <class ContainerAllocator>
        struct IsMessage< ::cube_pos::Robots_<ContainerAllocator> >
                : TrueType
        { };

        template <class ContainerAllocator>
        struct IsMessage< ::cube_pos::Robots_<ContainerAllocator> const>
                : TrueType
        { };

        template <class ContainerAllocator>
        struct HasHeader< ::cube_pos::Robots_<ContainerAllocator> >
                : TrueType
        { };

        template <class ContainerAllocator>
        struct HasHeader< ::cube_pos::Robots_<ContainerAllocator> const>
                : TrueType
        { };


        template<class ContainerAllocator>
        struct MD5Sum< ::cube_pos::Robots_<ContainerAllocator> >
        {
            static const char* value()
            {
                return "b7d13a113e41dad0cfea59f1c9d547a0";
            }

            static const char* value(const ::cube_pos::Robots_<ContainerAllocator>&) { return value(); }
            static const uint64_t static_value1 = 0xb7d13a113e41dad0ULL;
            static const uint64_t static_value2 = 0xcfea59f1c9d547a0ULL;
        };

        template<class ContainerAllocator>
        struct DataType< ::cube_pos::Robots_<ContainerAllocator> >
        {
            static const char* value()
            {
                return "cube_pos/Robots";
            }

            static const char* value(const ::cube_pos::Robots_<ContainerAllocator>&) { return value(); }
        };

        template<class ContainerAllocator>
        struct Definition< ::cube_pos::Robots_<ContainerAllocator> >
        {
            static const char* value()
            {
                return "Header header\n\
Robot[] robots\n\
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
MSG: cube_pos/Robot\n\
uint8 robot_id\n\
geometry_msgs/PoseWithCovarianceStamped pose\n\
geometry_msgs/TwistWithCovarianceStamped twist\n\
================================================================================\n\
MSG: geometry_msgs/PoseWithCovarianceStamped\n\
# This expresses an estimated pose with a reference coordinate frame and timestamp\n\
\n\
Header header\n\
PoseWithCovariance pose\n\
\n\
================================================================================\n\
MSG: geometry_msgs/PoseWithCovariance\n\
# This represents a pose in free space with uncertainty.\n\
\n\
Pose pose\n\
\n\
# Row-major representation of the 6x6 covariance matrix\n\
# The orientation parameters use a fixed-axis representation.\n\
# In order, the parameters are:\n\
# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)\n\
float64[36] covariance\n\
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
MSG: geometry_msgs/TwistWithCovarianceStamped\n\
# This represents an estimated twist with reference coordinate frame and timestamp.\n\
Header header\n\
TwistWithCovariance twist\n\
\n\
================================================================================\n\
MSG: geometry_msgs/TwistWithCovariance\n\
# This expresses velocity in free space with uncertainty.\n\
\n\
Twist twist\n\
\n\
# Row-major representation of the 6x6 covariance matrix\n\
# The orientation parameters use a fixed-axis representation.\n\
# In order, the parameters are:\n\
# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)\n\
float64[36] covariance\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Twist\n\
# This expresses velocity in free space broken into its linear and angular parts.\n\
Vector3  linear\n\
Vector3  angular\n\
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

            static const char* value(const ::cube_pos::Robots_<ContainerAllocator>&) { return value(); }
        };

    } // namespace message_traits
} // namespace ros

namespace ros
{
    namespace serialization
    {

        template<class ContainerAllocator> struct Serializer< ::cube_pos::Robots_<ContainerAllocator> >
        {
            template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
            {
                stream.next(m.header);
                stream.next(m.robots);
            }

            ROS_DECLARE_ALLINONE_SERIALIZER
        }; // struct Robots_

    } // namespace serialization
} // namespace ros

namespace ros
{
    namespace message_operations
    {

        template<class ContainerAllocator>
        struct Printer< ::cube_pos::Robots_<ContainerAllocator> >
        {
            template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::cube_pos::Robots_<ContainerAllocator>& v)
            {
                s << indent << "header: ";
                s << std::endl;
                Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
                s << indent << "robots[]" << std::endl;
                for (size_t i = 0; i < v.robots.size(); ++i)
                {
                    s << indent << "  robots[" << i << "]: ";
                    s << std::endl;
                    s << indent;
                    Printer< ::cube_pos::Robot_<ContainerAllocator> >::stream(s, indent + "    ", v.robots[i]);
                }
            }
        };

    } // namespace message_operations
} // namespace ros

#endif // CUBE_POS_MESSAGE_ROBOTS_H