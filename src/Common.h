#ifndef _COMMON_H_
#define _COMMON_H_

#include "Leg.h"

#ifndef ODE
#include "BNO055.h"
#endif

#include "SO1602A.h"
#include "PCA9685.h"
#include "PX4Flow.h"

/********************Mode***********************/
const int ALL_1(0);
const int ALL_2(1);
const int WALKING(2);
const int STAND(3);
const int SENSOR(4);
const int GERGE(5);
const int OFFSET(6);


/****************main.cpp*********************/
extern Serial pc;
extern DigitalIn UB;
extern I2C i2c;
extern BNO055 bno;
extern AnalogIn hand_sensor;
extern AnalogIn gerge_sensor;
extern SO1602A oled;
extern I2C optical_flow;
extern PCA9685 pwm;

extern DigitalInOut Xshut[2];

extern PX4Flow px4flow;

extern DigitalIn button_white;
extern DigitalIn button_red;
extern DigitalIn button_blue_left;
extern DigitalIn button_blue_right;

//extern PwmOut speaker;

extern DigitalIn micro_switch;
extern PwmOut gerge_1;
extern PwmOut gerge_2;

extern double roll, pitch, yaw;
extern double roll_offset, pitch_offset, yaw_offset;
extern double all_height_old;
extern int mode;
extern int state;
extern bool yaw_mode;
extern bool x_mode;
extern bool lidarlite_mode;
extern int pc_debug;
extern int oled_debug;
extern bool field;
extern bool leg_type;
extern double adjust_y;
extern bool attitude_control;

extern double yaw_ref;
extern double x_ref;
extern double x_now;
extern double y_now;
extern int16_t px;
extern int16_t py;

extern Leg rf;
extern Leg rb;
extern Leg lb;
extern Leg lf;

/***********************Output.cpp**************************/
extern double bodyposition[3];

/*************************Creeping.cpp********************/
extern double ratio_right;
extern double ratio_left;

/****************************************************/

#endif
