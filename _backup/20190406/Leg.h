#ifndef _LEG_H_
#define _LEG_H_

#include "Servo.h"

class Leg
{
public:
  double theta0, theta1, theta2; //モーター0,1,2の角度、Output_Coordinateで計算した結果の一時保管用
  double x, y, z;
  double adjust;
  void calc(double, double, double, double, double &, double &, double &);
  Leg(Servo &, Servo &, Servo &);
  Servo &s0;
  Servo &s1;
  Servo &s2;
};

#endif