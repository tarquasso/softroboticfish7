#ifndef _ROS_fish_msgs_mbedPumpStatusMsg_h
#define _ROS_fish_msgs_mbedPumpStatusMsg_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace fish_msgs
{

  class mbedPumpStatusMsg : public ros::Msg
  {
    public:
      int8_t mode;
      float yaw;
      float freq;

    mbedPumpStatusMsg():
      mode(0),
      yaw(0),
      freq(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int8_t real;
        uint8_t base;
      } u_mode;
      u_mode.real = this->mode;
      *(outbuffer + offset + 0) = (u_mode.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->mode);
      union {
        float real;
        uint32_t base;
      } u_yaw;
      u_yaw.real = this->yaw;
      *(outbuffer + offset + 0) = (u_yaw.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_yaw.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_yaw.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_yaw.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->yaw);
      union {
        float real;
        uint32_t base;
      } u_freq;
      u_freq.real = this->freq;
      *(outbuffer + offset + 0) = (u_freq.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_freq.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_freq.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_freq.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->freq);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int8_t real;
        uint8_t base;
      } u_mode;
      u_mode.base = 0;
      u_mode.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->mode = u_mode.real;
      offset += sizeof(this->mode);
      union {
        float real;
        uint32_t base;
      } u_yaw;
      u_yaw.base = 0;
      u_yaw.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_yaw.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_yaw.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_yaw.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->yaw = u_yaw.real;
      offset += sizeof(this->yaw);
      union {
        float real;
        uint32_t base;
      } u_freq;
      u_freq.base = 0;
      u_freq.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_freq.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_freq.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_freq.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->freq = u_freq.real;
      offset += sizeof(this->freq);
     return offset;
    }

    const char * getType(){ return "fish_msgs/mbedPumpStatusMsg"; };
    const char * getMD5(){ return "d362e13e6bb848f15ea16da4058b765d"; };

  };

}
#endif