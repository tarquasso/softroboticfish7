#ifndef _ROS_control_msgs_PidState_h
#define _ROS_control_msgs_PidState_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "ros/duration.h"

namespace control_msgs
{

  class PidState : public ros::Msg
  {
    public:
      std_msgs::Header header;
      ros::Duration timestep;
      double error;
      double error_dot;
      double p_error;
      double i_error;
      double d_error;
      double p_term;
      double i_term;
      double d_term;
      double i_max;
      double i_min;
      double output;

    PidState():
      header(),
      timestep(),
      error(0),
      error_dot(0),
      p_error(0),
      i_error(0),
      d_error(0),
      p_term(0),
      i_term(0),
      d_term(0),
      i_max(0),
      i_min(0),
      output(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->timestep.sec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->timestep.sec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->timestep.sec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->timestep.sec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->timestep.sec);
      *(outbuffer + offset + 0) = (this->timestep.nsec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->timestep.nsec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->timestep.nsec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->timestep.nsec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->timestep.nsec);
      union {
        double real;
        uint64_t base;
      } u_error;
      u_error.real = this->error;
      *(outbuffer + offset + 0) = (u_error.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_error.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_error.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_error.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_error.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_error.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_error.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_error.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->error);
      union {
        double real;
        uint64_t base;
      } u_error_dot;
      u_error_dot.real = this->error_dot;
      *(outbuffer + offset + 0) = (u_error_dot.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_error_dot.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_error_dot.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_error_dot.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_error_dot.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_error_dot.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_error_dot.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_error_dot.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->error_dot);
      union {
        double real;
        uint64_t base;
      } u_p_error;
      u_p_error.real = this->p_error;
      *(outbuffer + offset + 0) = (u_p_error.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_p_error.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_p_error.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_p_error.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_p_error.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_p_error.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_p_error.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_p_error.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->p_error);
      union {
        double real;
        uint64_t base;
      } u_i_error;
      u_i_error.real = this->i_error;
      *(outbuffer + offset + 0) = (u_i_error.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_i_error.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_i_error.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_i_error.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_i_error.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_i_error.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_i_error.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_i_error.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->i_error);
      union {
        double real;
        uint64_t base;
      } u_d_error;
      u_d_error.real = this->d_error;
      *(outbuffer + offset + 0) = (u_d_error.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_d_error.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_d_error.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_d_error.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_d_error.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_d_error.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_d_error.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_d_error.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->d_error);
      union {
        double real;
        uint64_t base;
      } u_p_term;
      u_p_term.real = this->p_term;
      *(outbuffer + offset + 0) = (u_p_term.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_p_term.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_p_term.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_p_term.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_p_term.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_p_term.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_p_term.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_p_term.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->p_term);
      union {
        double real;
        uint64_t base;
      } u_i_term;
      u_i_term.real = this->i_term;
      *(outbuffer + offset + 0) = (u_i_term.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_i_term.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_i_term.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_i_term.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_i_term.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_i_term.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_i_term.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_i_term.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->i_term);
      union {
        double real;
        uint64_t base;
      } u_d_term;
      u_d_term.real = this->d_term;
      *(outbuffer + offset + 0) = (u_d_term.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_d_term.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_d_term.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_d_term.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_d_term.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_d_term.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_d_term.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_d_term.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->d_term);
      union {
        double real;
        uint64_t base;
      } u_i_max;
      u_i_max.real = this->i_max;
      *(outbuffer + offset + 0) = (u_i_max.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_i_max.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_i_max.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_i_max.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_i_max.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_i_max.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_i_max.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_i_max.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->i_max);
      union {
        double real;
        uint64_t base;
      } u_i_min;
      u_i_min.real = this->i_min;
      *(outbuffer + offset + 0) = (u_i_min.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_i_min.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_i_min.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_i_min.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_i_min.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_i_min.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_i_min.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_i_min.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->i_min);
      union {
        double real;
        uint64_t base;
      } u_output;
      u_output.real = this->output;
      *(outbuffer + offset + 0) = (u_output.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_output.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_output.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_output.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_output.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_output.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_output.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_output.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->output);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      this->timestep.sec =  ((uint32_t) (*(inbuffer + offset)));
      this->timestep.sec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->timestep.sec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->timestep.sec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->timestep.sec);
      this->timestep.nsec =  ((uint32_t) (*(inbuffer + offset)));
      this->timestep.nsec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->timestep.nsec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->timestep.nsec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->timestep.nsec);
      union {
        double real;
        uint64_t base;
      } u_error;
      u_error.base = 0;
      u_error.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_error.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_error.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_error.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_error.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_error.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_error.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_error.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->error = u_error.real;
      offset += sizeof(this->error);
      union {
        double real;
        uint64_t base;
      } u_error_dot;
      u_error_dot.base = 0;
      u_error_dot.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_error_dot.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_error_dot.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_error_dot.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_error_dot.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_error_dot.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_error_dot.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_error_dot.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->error_dot = u_error_dot.real;
      offset += sizeof(this->error_dot);
      union {
        double real;
        uint64_t base;
      } u_p_error;
      u_p_error.base = 0;
      u_p_error.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_p_error.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_p_error.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_p_error.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_p_error.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_p_error.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_p_error.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_p_error.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->p_error = u_p_error.real;
      offset += sizeof(this->p_error);
      union {
        double real;
        uint64_t base;
      } u_i_error;
      u_i_error.base = 0;
      u_i_error.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_i_error.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_i_error.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_i_error.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_i_error.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_i_error.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_i_error.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_i_error.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->i_error = u_i_error.real;
      offset += sizeof(this->i_error);
      union {
        double real;
        uint64_t base;
      } u_d_error;
      u_d_error.base = 0;
      u_d_error.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_d_error.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_d_error.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_d_error.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_d_error.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_d_error.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_d_error.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_d_error.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->d_error = u_d_error.real;
      offset += sizeof(this->d_error);
      union {
        double real;
        uint64_t base;
      } u_p_term;
      u_p_term.base = 0;
      u_p_term.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_p_term.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_p_term.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_p_term.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_p_term.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_p_term.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_p_term.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_p_term.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->p_term = u_p_term.real;
      offset += sizeof(this->p_term);
      union {
        double real;
        uint64_t base;
      } u_i_term;
      u_i_term.base = 0;
      u_i_term.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_i_term.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_i_term.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_i_term.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_i_term.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_i_term.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_i_term.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_i_term.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->i_term = u_i_term.real;
      offset += sizeof(this->i_term);
      union {
        double real;
        uint64_t base;
      } u_d_term;
      u_d_term.base = 0;
      u_d_term.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_d_term.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_d_term.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_d_term.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_d_term.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_d_term.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_d_term.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_d_term.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->d_term = u_d_term.real;
      offset += sizeof(this->d_term);
      union {
        double real;
        uint64_t base;
      } u_i_max;
      u_i_max.base = 0;
      u_i_max.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_i_max.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_i_max.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_i_max.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_i_max.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_i_max.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_i_max.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_i_max.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->i_max = u_i_max.real;
      offset += sizeof(this->i_max);
      union {
        double real;
        uint64_t base;
      } u_i_min;
      u_i_min.base = 0;
      u_i_min.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_i_min.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_i_min.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_i_min.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_i_min.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_i_min.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_i_min.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_i_min.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->i_min = u_i_min.real;
      offset += sizeof(this->i_min);
      union {
        double real;
        uint64_t base;
      } u_output;
      u_output.base = 0;
      u_output.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_output.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_output.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_output.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_output.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_output.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_output.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_output.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->output = u_output.real;
      offset += sizeof(this->output);
     return offset;
    }

    const char * getType(){ return "control_msgs/PidState"; };
    const char * getMD5(){ return "b138ec00e886c10e73f27e8712252ea6"; };

  };

}
#endif