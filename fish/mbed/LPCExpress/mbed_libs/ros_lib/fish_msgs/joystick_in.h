#ifndef _ROS_fish_msgs_joystick_in_h
#define _ROS_fish_msgs_joystick_in_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace fish_msgs
{

  class joystick_in : public ros::Msg
  {
    public:
      int8_t freq_ctrl;
      int8_t speed_ctrl;
      int8_t depth_ctrl;
      int8_t yaw_ctrl;

    joystick_in():
      freq_ctrl(0),
      speed_ctrl(0),
      depth_ctrl(0),
      yaw_ctrl(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int8_t real;
        uint8_t base;
      } u_freq_ctrl;
      u_freq_ctrl.real = this->freq_ctrl;
      *(outbuffer + offset + 0) = (u_freq_ctrl.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->freq_ctrl);
      union {
        int8_t real;
        uint8_t base;
      } u_speed_ctrl;
      u_speed_ctrl.real = this->speed_ctrl;
      *(outbuffer + offset + 0) = (u_speed_ctrl.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->speed_ctrl);
      union {
        int8_t real;
        uint8_t base;
      } u_depth_ctrl;
      u_depth_ctrl.real = this->depth_ctrl;
      *(outbuffer + offset + 0) = (u_depth_ctrl.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->depth_ctrl);
      union {
        int8_t real;
        uint8_t base;
      } u_yaw_ctrl;
      u_yaw_ctrl.real = this->yaw_ctrl;
      *(outbuffer + offset + 0) = (u_yaw_ctrl.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->yaw_ctrl);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int8_t real;
        uint8_t base;
      } u_freq_ctrl;
      u_freq_ctrl.base = 0;
      u_freq_ctrl.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->freq_ctrl = u_freq_ctrl.real;
      offset += sizeof(this->freq_ctrl);
      union {
        int8_t real;
        uint8_t base;
      } u_speed_ctrl;
      u_speed_ctrl.base = 0;
      u_speed_ctrl.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->speed_ctrl = u_speed_ctrl.real;
      offset += sizeof(this->speed_ctrl);
      union {
        int8_t real;
        uint8_t base;
      } u_depth_ctrl;
      u_depth_ctrl.base = 0;
      u_depth_ctrl.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->depth_ctrl = u_depth_ctrl.real;
      offset += sizeof(this->depth_ctrl);
      union {
        int8_t real;
        uint8_t base;
      } u_yaw_ctrl;
      u_yaw_ctrl.base = 0;
      u_yaw_ctrl.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->yaw_ctrl = u_yaw_ctrl.real;
      offset += sizeof(this->yaw_ctrl);
     return offset;
    }

    const char * getType(){ return "fish_msgs/joystick_in"; };
    const char * getMD5(){ return "0c650c89727301b1e2298a1c19175b51"; };

  };

}
#endif