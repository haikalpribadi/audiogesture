/* Auto-generated by genmsg_cpp for file /home/haikalpribadi/Workspace/ROS/radiophonic/audiogesture/msg/PlayerCommand.msg */
#ifndef AUDIOGESTURE_MESSAGE_PLAYERCOMMAND_H
#define AUDIOGESTURE_MESSAGE_PLAYERCOMMAND_H
#include <string>
#include <vector>
#include <map>
#include <ostream>
#include "ros/serialization.h"
#include "ros/builtin_message_traits.h"
#include "ros/message_operations.h"
#include "ros/time.h"

#include "ros/macros.h"

#include "ros/assert.h"


namespace audiogesture
{
template <class ContainerAllocator>
struct PlayerCommand_ {
  typedef PlayerCommand_<ContainerAllocator> Type;

  PlayerCommand_()
  : name()
  , file()
  , command()
  {
  }

  PlayerCommand_(const ContainerAllocator& _alloc)
  : name(_alloc)
  , file(_alloc)
  , command(_alloc)
  {
  }

  typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _name_type;
  std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  name;

  typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _file_type;
  std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  file;

  typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _command_type;
  std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  command;


  typedef boost::shared_ptr< ::audiogesture::PlayerCommand_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::audiogesture::PlayerCommand_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct PlayerCommand
typedef  ::audiogesture::PlayerCommand_<std::allocator<void> > PlayerCommand;

typedef boost::shared_ptr< ::audiogesture::PlayerCommand> PlayerCommandPtr;
typedef boost::shared_ptr< ::audiogesture::PlayerCommand const> PlayerCommandConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::audiogesture::PlayerCommand_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::audiogesture::PlayerCommand_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace audiogesture

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::audiogesture::PlayerCommand_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::audiogesture::PlayerCommand_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::audiogesture::PlayerCommand_<ContainerAllocator> > {
  static const char* value() 
  {
    return "8b4d0ce774dae1224469c9b8fe027ac5";
  }

  static const char* value(const  ::audiogesture::PlayerCommand_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x8b4d0ce774dae122ULL;
  static const uint64_t static_value2 = 0x4469c9b8fe027ac5ULL;
};

template<class ContainerAllocator>
struct DataType< ::audiogesture::PlayerCommand_<ContainerAllocator> > {
  static const char* value() 
  {
    return "audiogesture/PlayerCommand";
  }

  static const char* value(const  ::audiogesture::PlayerCommand_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::audiogesture::PlayerCommand_<ContainerAllocator> > {
  static const char* value() 
  {
    return "string name\n\
string file\n\
string command\n\
\n\
";
  }

  static const char* value(const  ::audiogesture::PlayerCommand_<ContainerAllocator> &) { return value(); } 
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::audiogesture::PlayerCommand_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.name);
    stream.next(m.file);
    stream.next(m.command);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct PlayerCommand_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::audiogesture::PlayerCommand_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::audiogesture::PlayerCommand_<ContainerAllocator> & v) 
  {
    s << indent << "name: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.name);
    s << indent << "file: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.file);
    s << indent << "command: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.command);
  }
};


} // namespace message_operations
} // namespace ros

#endif // AUDIOGESTURE_MESSAGE_PLAYERCOMMAND_H
