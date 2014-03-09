/* Auto-generated by genmsg_cpp for file /home/haikalpribadi/Workspace/ROS/radiophonic/audiogesture/msg/ProcessedOutput.msg */
#ifndef AUDIOGESTURE_MESSAGE_PROCESSEDOUTPUT_H
#define AUDIOGESTURE_MESSAGE_PROCESSEDOUTPUT_H
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
struct ProcessedOutput_ {
  typedef ProcessedOutput_<ContainerAllocator> Type;

  ProcessedOutput_()
  : name()
  , type()
  , file()
  {
  }

  ProcessedOutput_(const ContainerAllocator& _alloc)
  : name(_alloc)
  , type(_alloc)
  , file(_alloc)
  {
  }

  typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _name_type;
  std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  name;

  typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _type_type;
  std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  type;

  typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _file_type;
  std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  file;


  typedef boost::shared_ptr< ::audiogesture::ProcessedOutput_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::audiogesture::ProcessedOutput_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct ProcessedOutput
typedef  ::audiogesture::ProcessedOutput_<std::allocator<void> > ProcessedOutput;

typedef boost::shared_ptr< ::audiogesture::ProcessedOutput> ProcessedOutputPtr;
typedef boost::shared_ptr< ::audiogesture::ProcessedOutput const> ProcessedOutputConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::audiogesture::ProcessedOutput_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::audiogesture::ProcessedOutput_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace audiogesture

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::audiogesture::ProcessedOutput_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::audiogesture::ProcessedOutput_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::audiogesture::ProcessedOutput_<ContainerAllocator> > {
  static const char* value() 
  {
    return "e1979c48e54b0e648564f0b60d474570";
  }

  static const char* value(const  ::audiogesture::ProcessedOutput_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xe1979c48e54b0e64ULL;
  static const uint64_t static_value2 = 0x8564f0b60d474570ULL;
};

template<class ContainerAllocator>
struct DataType< ::audiogesture::ProcessedOutput_<ContainerAllocator> > {
  static const char* value() 
  {
    return "audiogesture/ProcessedOutput";
  }

  static const char* value(const  ::audiogesture::ProcessedOutput_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::audiogesture::ProcessedOutput_<ContainerAllocator> > {
  static const char* value() 
  {
    return "string name\n\
string type\n\
string file\n\
\n\
";
  }

  static const char* value(const  ::audiogesture::ProcessedOutput_<ContainerAllocator> &) { return value(); } 
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::audiogesture::ProcessedOutput_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.name);
    stream.next(m.type);
    stream.next(m.file);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct ProcessedOutput_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::audiogesture::ProcessedOutput_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::audiogesture::ProcessedOutput_<ContainerAllocator> & v) 
  {
    s << indent << "name: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.name);
    s << indent << "type: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.type);
    s << indent << "file: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.file);
  }
};


} // namespace message_operations
} // namespace ros

#endif // AUDIOGESTURE_MESSAGE_PROCESSEDOUTPUT_H

