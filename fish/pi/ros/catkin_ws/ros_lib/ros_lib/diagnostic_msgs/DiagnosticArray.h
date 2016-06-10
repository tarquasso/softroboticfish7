#ifndef _ROS_diagnostic_msgs_DiagnosticArray_h
#define _ROS_diagnostic_msgs_DiagnosticArray_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "diagnostic_msgs/DiagnosticStatus.h"

namespace diagnostic_msgs
{

  class DiagnosticArray : public ros::Msg
  {
    public:
      std_msgs::Header header;
      uint32_t status_length;
      diagnostic_msgs::DiagnosticStatus st_status;
      diagnostic_msgs::DiagnosticStatus * status;

    DiagnosticArray():
      header(),
      status_length(0), status(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->status_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->status_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->status_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->status_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->status_length);
      for( uint32_t i = 0; i < status_length; i++){
      offset += this->status[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t status_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      status_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      status_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      status_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->status_length);
      if(status_lengthT > status_length)
        this->status = (diagnostic_msgs::DiagnosticStatus*)realloc(this->status, status_lengthT * sizeof(diagnostic_msgs::DiagnosticStatus));
      status_length = status_lengthT;
      for( uint32_t i = 0; i < status_length; i++){
      offset += this->st_status.deserialize(inbuffer + offset);
        memcpy( &(this->status[i]), &(this->st_status), sizeof(diagnostic_msgs::DiagnosticStatus));
      }
     return offset;
    }

    const char * getType(){ return "diagnostic_msgs/DiagnosticArray"; };
    const char * getMD5(){ return "60810da900de1dd6ddd437c3503511da"; };

  };

}
#endif