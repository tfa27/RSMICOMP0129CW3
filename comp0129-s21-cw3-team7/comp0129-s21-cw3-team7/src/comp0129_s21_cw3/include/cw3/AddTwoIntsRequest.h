/* Software License Agreement (BSD License)
 *
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *  * Neither the name of Willow Garage, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef BEGINNER_TUTORIALS_MESSAGE_ADDTWOINTSREQUEST_H
#define BEGINNER_TUTORIALS_MESSAGE_ADDTWOINTSREQUEST_H

#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace beginner_tutorials
{
  template <class ContainerAllocator>
  struct AddTwoIntsRequest_
  {
    typedef AddTwoIntsRequest_<ContainerAllocator> Type;

    AddTwoIntsRequest_()
      : a(0)
      , b(0)  {
      }
    AddTwoIntsRequest_(const ContainerAllocator& _alloc)
      : a(0)
      , b(0)  {
      }
      
     typedef int64_t _a_type;
    _a_type a;
    
     typedef int64_t _b_type;
    _b_type b;
    
    typedef boost::shared_ptr< ::beginner_tutorials::AddTwoIntsRequest_<ContainerAllocator> > Ptr;
    typedef boost::shared_ptr< ::beginner_tutorials::AddTwoIntsRequest_<ContainerAllocator> const> ConstPtr;
  }; // struct AddTwoIntsRequest_

  typedef ::beginner_tutorials::AddTwoIntsRequest_<std::allocator<void> > AddTwoIntsRequest;
  typedef boost::shared_ptr< ::beginner_tutorials::AddTwoIntsRequest > AddTwoIntsRequestPtr;
  typedef boost::shared_ptr< ::beginner_tutorials::AddTwoIntsRequest const> AddTwoIntsRequestConstPtr;
  
  // constants requiring out of line definition
  template<typename ContainerAllocator>
  std::ostream& operator<<(std::ostream& s, const ::beginner_tutorials::AddTwoIntsRequest_<ContainerAllocator> & v)
  {
    ros::message_operations::Printer< ::beginner_tutorials::AddTwoIntsRequest_<ContainerAllocator> >::stream(s, "", v);
    return s;
  }
} // namespace beginner_tutorials

namespace ros
{
  namespace message_traits
  {
    template <class ContainerAllocator>
    struct IsFixedSize< ::beginner_tutorials::AddTwoIntsRequest_<ContainerAllocator> >
      : TrueType
      { };

    template <class ContainerAllocator>
    struct IsFixedSize< ::beginner_tutorials::AddTwoIntsRequest_<ContainerAllocator> const>
      : TrueType
      { };

    template <class ContainerAllocator>
    struct IsMessage< ::beginner_tutorials::AddTwoIntsRequest_<ContainerAllocator> >
      : TrueType
      { };

    template <class ContainerAllocator>
    struct IsMessage< ::beginner_tutorials::AddTwoIntsRequest_<ContainerAllocator> const>
      : TrueType
      { };

    template <class ContainerAllocator>
    struct HasHeader< ::beginner_tutorials::AddTwoIntsRequest_<ContainerAllocator> >
      : FalseType
      { };

    template <class ContainerAllocator>
    struct HasHeader< ::beginner_tutorials::AddTwoIntsRequest_<ContainerAllocator> const>
      : FalseType
      { };


    template<class ContainerAllocator>
    struct MD5Sum< ::beginner_tutorials::AddTwoIntsRequest_<ContainerAllocator> >
    {
      static const char* value()
      {
        return "36d09b846be0b371c5f190354dd3153e";
      }

      static const char* value(const ::beginner_tutorials::AddTwoIntsRequest_<ContainerAllocator>&) { return value(); }
      static const uint64_t static_value1 = 0x36d09b846be0b371ULL;
      static const uint64_t static_value2 = 0xc5f190354dd3153eULL;
    };

    template<class ContainerAllocator>
    struct DataType< ::beginner_tutorials::AddTwoIntsRequest_<ContainerAllocator> >
    {
      static const char* value()
      {
        return "beginner_tutorials/AddTwoIntsRequest";
      }

      static const char* value(const ::beginner_tutorials::AddTwoIntsRequest_<ContainerAllocator>&) { return value(); }
    };

    template<class ContainerAllocator>
    struct Definition< ::beginner_tutorials::AddTwoIntsRequest_<ContainerAllocator> >
    {
      static const char* value()
      {
        return "int64 a\n\
    int64 b\n\
    ";
      }

      static const char* value(const ::beginner_tutorials::AddTwoIntsRequest_<ContainerAllocator>&) { return value(); }
    };

  } // namespace message_traits
} // namespace ros

namespace ros
{
  namespace serialization
  {
    template<class ContainerAllocator> struct Serializer< ::beginner_tutorials::AddTwoIntsRequest_<ContainerAllocator> >
    {
      template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
      {
        stream.next(m.a);
        stream.next(m.b);
      }
      
      ROS_DECLARE_ALLINONE_SERIALIZER;
    }; // struct AddTwoIntsRequest_
  } // namespace serialization
} // namespace ros

namespace ros
{
  namespace message_operations
  {
    template<class ContainerAllocator>
    struct Printer< ::beginner_tutorials::AddTwoIntsRequest_<ContainerAllocator> >
    {
      template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::beginner_tutorials::AddTwoIntsRequest_<ContainerAllocator>& v)
      {
        s << indent << "a: ";
        Printer<int64_t>::stream(s, indent + "  ", v.a);
        s << indent << "b: ";
        Printer<int64_t>::stream(s, indent + "  ", v.b);
      }
    };
  } // namespace message_operations
} // namespace ros

#endif // BEGINNER_TUTORIALS_MESSAGE_ADDTWOINTSREQUEST_H