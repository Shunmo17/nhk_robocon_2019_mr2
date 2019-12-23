#ifndef _FUNCTION_H_
#define _FUNCTION_H_

#include "Common.h"

void Set_Angle_Offset(double, double, double,
                      double, double, double,
                      double, double, double,
                      double, double, double);

void Set_Angle_Min(double, double, double,
                   double, double, double,
                   double, double, double,
                   double, double, double);

void Set_Angle_Max(double, double, double,
                   double, double, double,
                   double, double, double,
                   double, double, double);

void Wait_UB(void);

void Body_Up(double, double, double, double);

void Body_Up_Dainozi(double, double, double);

void Body_Down(double, double);

void Hip_Shaking(double, double, double,
                 double, int);

void Offset_Position(void);

void Store_Position(void);

void Set_Gyro_Offset(void);

double Hand_distance(void);

void setServoPulse(uint8_t, float);

void initServoDriver(void);

void Position_estimate(double);

#endif