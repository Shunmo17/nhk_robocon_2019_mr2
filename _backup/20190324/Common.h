#ifndef _COMMON_H_
#define _COMMON_H_

#include "Leg.h"

#ifndef ODE
#include "BNO055.h"
#endif

#include "SO1602A.h"
#include "PCA9685.h"
#include "VL53L0X.h"

/****************main.cpp*********************/
extern Serial pc;
extern DigitalIn UB;
extern I2C i2c;
extern BNO055 bno;
extern AnalogIn distance_sensor;
extern SO1602A oled;
extern PCA9685 pwm;

extern Timer timer_vl[2]; //VL53L0x
extern VL53L0X vl[2];
extern DigitalInOut Xshut[2];

extern DigitalIn button_white;
extern DigitalIn button_red;
extern DigitalIn button_blue_left;
extern DigitalIn button_blue_right;

extern double roll, pitch, yaw;
extern double roll_offset, pitch_offset, yaw_offset;
extern double all_height_old;
extern int mode;
extern bool yaw_mode;
extern int pc_debug;
extern int oled_debug;
extern int vl_output[2];
extern double vl_distance[2];
extern double yaw_old;
extern double yaw_ref;

extern Leg rf;
extern Leg rb;
extern Leg lb;
extern Leg lf;

extern PwmOut speaker;

/***********************Output.cpp**************************/
extern double bodyposition[3];
/****************************************************/

#endif
