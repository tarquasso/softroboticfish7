#ifndef _ROS_SERVICE_SegmentImage_h
#define _ROS_SERVICE_SegmentImage_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "sensor_msgs/Image.h"
#include "duckietown_msgs/SceneSegments.h"

namespace scene_segmentation
{

static const char SEGMENTIMAGE[] = "scene_segmentation/SegmentImage";

  class SegmentImageRequest : public ros::Msg
  {
    public:
      sensor_msgs::Image image;

    SegmentImageRequest():
      image()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->image.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->image.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return SEGMENTIMAGE; };
    const char * getMD5(){ return "b13d2865c5af2a64e6e30ab1b56e1dd5"; };

  };

  class SegmentImageResponse : public ros::Msg
  {
    public:
      duckietown_msgs::SceneSegments ss;

    SegmentImageResponse():
      ss()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->ss.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->ss.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return SEGMENTIMAGE; };
    const char * getMD5(){ return "8526730ee754ca6bc61ac5f2d468be49"; };

  };

  class SegmentImage {
    public:
    typedef SegmentImageRequest Request;
    typedef SegmentImageResponse Response;
  };

}
#endif
