#ifndef _CREEPING_H_
#define _CREEPING_H_

#define X0 0.15
#define Y0 0.15
#define Y1 0.016
#define ALL_HEIGHT 0.27
#define ADJUST_Y 0.02
#define STEP_LENGTH 0.35//0.40
#define STEP_HEIGHT 0.20
#define CREEPING_STEP_TIMES 5
#define CREEPING_WAIT_TIME 0.004

void Check_Yaw(double);
void Check_y(double);

void Creeping_repeat_2step(double, double, double, double, double,
                           double, double,
                           double, double);

void Creeping_repeat_1step(double, double, double, double, double,
                           double, double,
                           double, double);

void Creeping(double, double, double, double, double,
              double, double,
              double, double,
              int);

void LegType0_to_Dainozi();
void LegType1_to_Dainozi();
void Dainozi_to_LegType0();

#endif