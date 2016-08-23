#ifndef _ROS_fish_msgs_PumpTestMsg_h
#define _ROS_fish_msgs_PumpTestMsg_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace fish_msgs
{

  class PumpTestMsg : public ros::Msg
  {
    public:
      int8_t mode;
      float freq;
      float yaw;

    PumpTestMsg():
      mode(0),
      freq(0),
      yaw(0)
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
      } u_freq;
      u_freq.real = this->freq;
      *(outbuffer + offset + 0) = (u_freq.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_freq.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_freq.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_freq.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->freq);
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
      } u_freq;
      u_freq.base = 0;
      u_freq.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_freq.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_freq.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_freq.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->freq = u_freq.real;
      offset += sizeof(this->freq);
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
     return offset;
    }

    const char * getType(){ return "fish_msgs/PumpTestMsg"; };
    const char * getMD5(){ return "34fadd9771870a86de960e911d1d8c17"; };

  };

}
#endif