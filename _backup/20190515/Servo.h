#ifndef _SERVO_H
#define _SERVO_H

#include "Setting.h"

#ifndef ODE
#include <mbed.h>
#endif

class Servo
{
public:
  Servo(void);
  double angle_min;    //最小角度[deg]
  double angle_max;    //最大角度[deg]
  double angle_offset; //基準角度[deg]
  double angle_old;
  double angle_ref;
  double angle_now;
  int angle(double); //角度指令値からパルス幅を計算
};

#endif