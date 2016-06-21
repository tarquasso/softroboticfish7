#ifndef _ROS_opencv_apps_Moment_h
#define _ROS_opencv_apps_Moment_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "opencv_apps/Point2D.h"

namespace opencv_apps
{

  class Moment : public ros::Msg
  {
    public:
      double m00;
      double m10;
      double m01;
      double m20;
      double m11;
      double m02;
      double m30;
      double m21;
      double m12;
      double m03;
      double mu20;
      double mu11;
      double mu02;
      double mu30;
      double mu21;
      double mu12;
      double mu03;
      double nu20;
      double nu11;
      double nu02;
      double nu30;
      double nu21;
      double nu12;
      double nu03;
      opencv_apps::Point2D center;
      double length;
      double area;

    Moment():
      m00(0),
      m10(0),
      m01(0),
      m20(0),
      m11(0),
      m02(0),
      m30(0),
      m21(0),
      m12(0),
      m03(0),
      mu20(0),
      mu11(0),
      mu02(0),
      mu30(0),
      mu21(0),
      mu12(0),
      mu03(0),
      nu20(0),
      nu11(0),
      nu02(0),
      nu30(0),
      nu21(0),
      nu12(0),
      nu03(0),
      center(),
      length(0),
      area(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        double real;
        uint64_t base;
      } u_m00;
      u_m00.real = this->m00;
      *(outbuffer + offset + 0) = (u_m00.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_m00.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_m00.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_m00.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_m00.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_m00.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_m00.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_m00.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->m00);
      union {
        double real;
        uint64_t base;
      } u_m10;
      u_m10.real = this->m10;
      *(outbuffer + offset + 0) = (u_m10.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_m10.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_m10.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_m10.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_m10.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_m10.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_m10.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_m10.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->m10);
      union {
        double real;
        uint64_t base;
      } u_m01;
      u_m01.real = this->m01;
      *(outbuffer + offset + 0) = (u_m01.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_m01.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_m01.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_m01.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_m01.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_m01.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_m01.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_m01.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->m01);
      union {
        double real;
        uint64_t base;
      } u_m20;
      u_m20.real = this->m20;
      *(outbuffer + offset + 0) = (u_m20.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_m20.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_m20.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_m20.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_m20.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_m20.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_m20.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_m20.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->m20);
      union {
        double real;
        uint64_t base;
      } u_m11;
      u_m11.real = this->m11;
      *(outbuffer + offset + 0) = (u_m11.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_m11.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_m11.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_m11.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_m11.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_m11.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_m11.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_m11.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->m11);
      union {
        double real;
        uint64_t base;
      } u_m02;
      u_m02.real = this->m02;
      *(outbuffer + offset + 0) = (u_m02.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_m02.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_m02.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_m02.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_m02.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_m02.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_m02.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_m02.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->m02);
      union {
        double real;
        uint64_t base;
      } u_m30;
      u_m30.real = this->m30;
      *(outbuffer + offset + 0) = (u_m30.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_m30.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_m30.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_m30.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_m30.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_m30.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_m30.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_m30.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->m30);
      union {
        double real;
        uint64_t base;
      } u_m21;
      u_m21.real = this->m21;
      *(outbuffer + offset + 0) = (u_m21.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_m21.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_m21.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_m21.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_m21.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_m21.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_m21.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_m21.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->m21);
      union {
        double real;
        uint64_t base;
      } u_m12;
      u_m12.real = this->m12;
      *(outbuffer + offset + 0) = (u_m12.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_m12.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_m12.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_m12.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_m12.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_m12.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_m12.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_m12.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->m12);
      union {
        double real;
        uint64_t base;
      } u_m03;
      u_m03.real = this->m03;
      *(outbuffer + offset + 0) = (u_m03.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_m03.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_m03.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_m03.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_m03.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_m03.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_m03.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_m03.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->m03);
      union {
        double real;
        uint64_t base;
      } u_mu20;
      u_mu20.real = this->mu20;
      *(outbuffer + offset + 0) = (u_mu20.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_mu20.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_mu20.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_mu20.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_mu20.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_mu20.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_mu20.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_mu20.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->mu20);
      union {
        double real;
        uint64_t base;
      } u_mu11;
      u_mu11.real = this->mu11;
      *(outbuffer + offset + 0) = (u_mu11.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_mu11.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_mu11.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_mu11.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_mu11.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_mu11.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_mu11.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_mu11.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->mu11);
      union {
        double real;
        uint64_t base;
      } u_mu02;
      u_mu02.real = this->mu02;
      *(outbuffer + offset + 0) = (u_mu02.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_mu02.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_mu02.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_mu02.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_mu02.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_mu02.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_mu02.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_mu02.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->mu02);
      union {
        double real;
        uint64_t base;
      } u_mu30;
      u_mu30.real = this->mu30;
      *(outbuffer + offset + 0) = (u_mu30.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_mu30.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_mu30.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_mu30.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_mu30.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_mu30.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_mu30.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_mu30.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->mu30);
      union {
        double real;
        uint64_t base;
      } u_mu21;
      u_mu21.real = this->mu21;
      *(outbuffer + offset + 0) = (u_mu21.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_mu21.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_mu21.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_mu21.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_mu21.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_mu21.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_mu21.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_mu21.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->mu21);
      union {
        double real;
        uint64_t base;
      } u_mu12;
      u_mu12.real = this->mu12;
      *(outbuffer + offset + 0) = (u_mu12.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_mu12.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_mu12.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_mu12.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_mu12.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_mu12.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_mu12.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_mu12.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->mu12);
      union {
        double real;
        uint64_t base;
      } u_mu03;
      u_mu03.real = this->mu03;
      *(outbuffer + offset + 0) = (u_mu03.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_mu03.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_mu03.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_mu03.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_mu03.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_mu03.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_mu03.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_mu03.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->mu03);
      union {
        double real;
        uint64_t base;
      } u_nu20;
      u_nu20.real = this->nu20;
      *(outbuffer + offset + 0) = (u_nu20.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_nu20.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_nu20.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_nu20.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_nu20.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_nu20.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_nu20.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_nu20.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->nu20);
      union {
        double real;
        uint64_t base;
      } u_nu11;
      u_nu11.real = this->nu11;
      *(outbuffer + offset + 0) = (u_nu11.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_nu11.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_nu11.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_nu11.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_nu11.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_nu11.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_nu11.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_nu11.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->nu11);
      union {
        double real;
        uint64_t base;
      } u_nu02;
      u_nu02.real = this->nu02;
      *(outbuffer + offset + 0) = (u_nu02.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_nu02.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_nu02.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_nu02.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_nu02.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_nu02.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_nu02.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_nu02.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->nu02);
      union {
        double real;
        uint64_t base;
      } u_nu30;
      u_nu30.real = this->nu30;
      *(outbuffer + offset + 0) = (u_nu30.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_nu30.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_nu30.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_nu30.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_nu30.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_nu30.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_nu30.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_nu30.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->nu30);
      union {
        double real;
        uint64_t base;
      } u_nu21;
      u_nu21.real = this->nu21;
      *(outbuffer + offset + 0) = (u_nu21.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_nu21.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_nu21.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_nu21.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_nu21.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_nu21.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_nu21.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_nu21.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->nu21);
      union {
        double real;
        uint64_t base;
      } u_nu12;
      u_nu12.real = this->nu12;
      *(outbuffer + offset + 0) = (u_nu12.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_nu12.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_nu12.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_nu12.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_nu12.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_nu12.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_nu12.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_nu12.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->nu12);
      union {
        double real;
        uint64_t base;
      } u_nu03;
      u_nu03.real = this->nu03;
      *(outbuffer + offset + 0) = (u_nu03.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_nu03.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_nu03.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_nu03.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_nu03.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_nu03.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_nu03.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_nu03.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->nu03);
      offset += this->center.serialize(outbuffer + offset);
      union {
        double real;
        uint64_t base;
      } u_length;
      u_length.real = this->length;
      *(outbuffer + offset + 0) = (u_length.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_length.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_length.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_length.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_length.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_length.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_length.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_length.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->length);
      union {
        double real;
        uint64_t base;
      } u_area;
      u_area.real = this->area;
      *(outbuffer + offset + 0) = (u_area.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_area.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_area.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_area.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_area.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_area.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_area.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_area.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->area);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        double real;
        uint64_t base;
      } u_m00;
      u_m00.base = 0;
      u_m00.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_m00.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_m00.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_m00.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_m00.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_m00.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_m00.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_m00.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->m00 = u_m00.real;
      offset += sizeof(this->m00);
      union {
        double real;
        uint64_t base;
      } u_m10;
      u_m10.base = 0;
      u_m10.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_m10.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_m10.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_m10.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_m10.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_m10.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_m10.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_m10.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->m10 = u_m10.real;
      offset += sizeof(this->m10);
      union {
        double real;
        uint64_t base;
      } u_m01;
      u_m01.base = 0;
      u_m01.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_m01.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_m01.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_m01.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_m01.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_m01.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_m01.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_m01.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->m01 = u_m01.real;
      offset += sizeof(this->m01);
      union {
        double real;
        uint64_t base;
      } u_m20;
      u_m20.base = 0;
      u_m20.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_m20.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_m20.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_m20.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_m20.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_m20.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_m20.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_m20.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->m20 = u_m20.real;
      offset += sizeof(this->m20);
      union {
        double real;
        uint64_t base;
      } u_m11;
      u_m11.base = 0;
      u_m11.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_m11.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_m11.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_m11.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_m11.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_m11.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_m11.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_m11.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->m11 = u_m11.real;
      offset += sizeof(this->m11);
      union {
        double real;
        uint64_t base;
      } u_m02;
      u_m02.base = 0;
      u_m02.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_m02.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_m02.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_m02.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_m02.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_m02.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_m02.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_m02.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->m02 = u_m02.real;
      offset += sizeof(this->m02);
      union {
        double real;
        uint64_t base;
      } u_m30;
      u_m30.base = 0;
      u_m30.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_m30.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_m30.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_m30.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_m30.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_m30.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_m30.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_m30.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->m30 = u_m30.real;
      offset += sizeof(this->m30);
      union {
        double real;
        uint64_t base;
      } u_m21;
      u_m21.base = 0;
      u_m21.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_m21.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_m21.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_m21.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_m21.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_m21.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_m21.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_m21.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->m21 = u_m21.real;
      offset += sizeof(this->m21);
      union {
        double real;
        uint64_t base;
      } u_m12;
      u_m12.base = 0;
      u_m12.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_m12.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_m12.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_m12.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_m12.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_m12.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_m12.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_m12.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->m12 = u_m12.real;
      offset += sizeof(this->m12);
      union {
        double real;
        uint64_t base;
      } u_m03;
      u_m03.base = 0;
      u_m03.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_m03.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_m03.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_m03.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_m03.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_m03.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_m03.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_m03.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->m03 = u_m03.real;
      offset += sizeof(this->m03);
      union {
        double real;
        uint64_t base;
      } u_mu20;
      u_mu20.base = 0;
      u_mu20.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_mu20.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_mu20.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_mu20.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_mu20.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_mu20.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_mu20.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_mu20.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->mu20 = u_mu20.real;
      offset += sizeof(this->mu20);
      union {
        double real;
        uint64_t base;
      } u_mu11;
      u_mu11.base = 0;
      u_mu11.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_mu11.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_mu11.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_mu11.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_mu11.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_mu11.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_mu11.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_mu11.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->mu11 = u_mu11.real;
      offset += sizeof(this->mu11);
      union {
        double real;
        uint64_t base;
      } u_mu02;
      u_mu02.base = 0;
      u_mu02.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_mu02.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_mu02.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_mu02.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_mu02.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_mu02.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_mu02.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_mu02.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->mu02 = u_mu02.real;
      offset += sizeof(this->mu02);
      union {
        double real;
        uint64_t base;
      } u_mu30;
      u_mu30.base = 0;
      u_mu30.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_mu30.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_mu30.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_mu30.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_mu30.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_mu30.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_mu30.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_mu30.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->mu30 = u_mu30.real;
      offset += sizeof(this->mu30);
      union {
        double real;
        uint64_t base;
      } u_mu21;
      u_mu21.base = 0;
      u_mu21.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_mu21.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_mu21.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_mu21.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_mu21.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_mu21.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_mu21.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_mu21.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->mu21 = u_mu21.real;
      offset += sizeof(this->mu21);
      union {
        double real;
        uint64_t base;
      } u_mu12;
      u_mu12.base = 0;
      u_mu12.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_mu12.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_mu12.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_mu12.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_mu12.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_mu12.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_mu12.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_mu12.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->mu12 = u_mu12.real;
      offset += sizeof(this->mu12);
      union {
        double real;
        uint64_t base;
      } u_mu03;
      u_mu03.base = 0;
      u_mu03.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_mu03.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_mu03.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_mu03.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_mu03.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_mu03.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_mu03.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_mu03.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->mu03 = u_mu03.real;
      offset += sizeof(this->mu03);
      union {
        double real;
        uint64_t base;
      } u_nu20;
      u_nu20.base = 0;
      u_nu20.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_nu20.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_nu20.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_nu20.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_nu20.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_nu20.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_nu20.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_nu20.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->nu20 = u_nu20.real;
      offset += sizeof(this->nu20);
      union {
        double real;
        uint64_t base;
      } u_nu11;
      u_nu11.base = 0;
      u_nu11.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_nu11.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_nu11.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_nu11.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_nu11.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_nu11.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_nu11.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_nu11.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->nu11 = u_nu11.real;
      offset += sizeof(this->nu11);
      union {
        double real;
        uint64_t base;
      } u_nu02;
      u_nu02.base = 0;
      u_nu02.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_nu02.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_nu02.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_nu02.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_nu02.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_nu02.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_nu02.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_nu02.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->nu02 = u_nu02.real;
      offset += sizeof(this->nu02);
      union {
        double real;
        uint64_t base;
      } u_nu30;
      u_nu30.base = 0;
      u_nu30.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_nu30.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_nu30.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_nu30.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_nu30.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_nu30.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_nu30.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_nu30.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->nu30 = u_nu30.real;
      offset += sizeof(this->nu30);
      union {
        double real;
        uint64_t base;
      } u_nu21;
      u_nu21.base = 0;
      u_nu21.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_nu21.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_nu21.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_nu21.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_nu21.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_nu21.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_nu21.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_nu21.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->nu21 = u_nu21.real;
      offset += sizeof(this->nu21);
      union {
        double real;
        uint64_t base;
      } u_nu12;
      u_nu12.base = 0;
      u_nu12.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_nu12.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_nu12.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_nu12.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_nu12.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_nu12.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_nu12.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_nu12.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->nu12 = u_nu12.real;
      offset += sizeof(this->nu12);
      union {
        double real;
        uint64_t base;
      } u_nu03;
      u_nu03.base = 0;
      u_nu03.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_nu03.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_nu03.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_nu03.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_nu03.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_nu03.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_nu03.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_nu03.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->nu03 = u_nu03.real;
      offset += sizeof(this->nu03);
      offset += this->center.deserialize(inbuffer + offset);
      union {
        double real;
        uint64_t base;
      } u_length;
      u_length.base = 0;
      u_length.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_length.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_length.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_length.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_length.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_length.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_length.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_length.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->length = u_length.real;
      offset += sizeof(this->length);
      union {
        double real;
        uint64_t base;
      } u_area;
      u_area.base = 0;
      u_area.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_area.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_area.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_area.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_area.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_area.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_area.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_area.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->area = u_area.real;
      offset += sizeof(this->area);
     return offset;
    }

    const char * getType(){ return "opencv_apps/Moment"; };
    const char * getMD5(){ return "560ee3fabfffb4ed4155742d6db8a03c"; };

  };

}
#endif