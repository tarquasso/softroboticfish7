#ifndef _ROS_opencv_apps_RotatedRect_h
#define _ROS_opencv_apps_RotatedRect_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "opencv_apps/Point2D.h"
#include "opencv_apps/Size.h"

namespace opencv_apps
{

  class RotatedRect : public ros::Msg
  {
    public:
      double angle;
      opencv_apps::Point2D center;
      opencv_apps::Size size;

    RotatedRect():
      angle(0),
      center(),
      size()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        double real;
        uint64_t base;
      } u_angle;
      u_angle.real = this->angle;
      *(outbuffer + offset + 0) = (u_angle.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_angle.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_angle.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_angle.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_angle.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_angle.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_angle.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_angle.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->angle);
      offset += this->center.serialize(outbuffer + offset);
      offset += this->size.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        double real;
        uint64_t base;
      } u_angle;
      u_angle.base = 0;
      u_angle.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_angle.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_angle.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_angle.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_angle.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_angle.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_angle.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_angle.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->angle = u_angle.real;
      offset += sizeof(this->angle);
      offset += this->center.deserialize(inbuffer + offset);
      offset += this->size.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "opencv_apps/RotatedRect"; };
    const char * getMD5(){ return "0ae60505c52f020176686d0689b8d390"; };

  };

}
#endif