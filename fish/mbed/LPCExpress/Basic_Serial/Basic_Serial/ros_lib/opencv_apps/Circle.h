#ifndef _ROS_opencv_apps_Circle_h
#define _ROS_opencv_apps_Circle_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "opencv_apps/Point2D.h"

namespace opencv_apps
{

  class Circle : public ros::Msg
  {
    public:
      opencv_apps::Point2D center;
      double radius;

    Circle():
      center(),
      radius(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->center.serialize(outbuffer + offset);
      union {
        double real;
        uint64_t base;
      } u_radius;
      u_radius.real = this->radius;
      *(outbuffer + offset + 0) = (u_radius.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_radius.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_radius.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_radius.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_radius.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_radius.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_radius.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_radius.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->radius);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->center.deserialize(inbuffer + offset);
      union {
        double real;
        uint64_t base;
      } u_radius;
      u_radius.base = 0;
      u_radius.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_radius.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_radius.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_radius.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_radius.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_radius.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_radius.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_radius.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->radius = u_radius.real;
      offset += sizeof(this->radius);
     return offset;
    }

    const char * getType(){ return "opencv_apps/Circle"; };
    const char * getMD5(){ return "4f6847051b4fe493b5af8caad66201d5"; };

  };

}
#endif