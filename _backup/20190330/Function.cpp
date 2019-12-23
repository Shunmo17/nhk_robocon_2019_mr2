#include "Common.h"
#include "Output.h"
#include "SO1602A.h"

#define mC 261.626
#define mD 293.665
#define mE 329.628
#define mF 349.228
#define mG 391.995
#define mA 440.000
#define mB 493.883

/****************************オフセット設定関数***************************/
void Set_Angle_Offset(double offset_a_s0, double offset_a_s1, double offset_a_s2,
                      double offset_b_s0, double offset_b_s1, double offset_b_s2,
                      double offset_c_s0, double offset_c_s1, double offset_c_s2,
                      double offset_d_s0, double offset_d_s1, double offset_d_s2)
{
  int j = 1;
#ifdef ODE
  j = 0; // ODEの時はオフセットをゼロに設定する
#endif
  rf.s0.angle_offset = offset_a_s0 * j;
  rf.s1.angle_offset = offset_a_s1 * j;
  rf.s2.angle_offset = offset_a_s2 * j;
  rb.s0.angle_offset = offset_b_s0 * j;
  rb.s1.angle_offset = offset_b_s1 * j;
  rb.s2.angle_offset = offset_b_s2 * j;
  lb.s0.angle_offset = offset_c_s0 * j;
  lb.s1.angle_offset = offset_c_s1 * j;
  lb.s2.angle_offset = offset_c_s2 * j;
  lf.s0.angle_offset = offset_d_s0 * j;
  lf.s1.angle_offset = offset_d_s1 * j;
  lf.s2.angle_offset = offset_d_s2 * j;
}

/****************************最小角度設定関数***************************/
void Set_Angle_Min(double min_a_s0, double min_a_s1, double min_a_s2,
                   double min_b_s0, double min_b_s1, double min_b_s2,
                   double min_c_s0, double min_c_s1, double min_c_s2,
                   double min_d_s0, double min_d_s1, double min_d_s2)
{
  rf.s0.angle_min = min_a_s0;
  rf.s1.angle_min = min_a_s1;
  rf.s2.angle_min = min_a_s2;
  rb.s0.angle_min = min_b_s0;
  rb.s1.angle_min = min_b_s1;
  rb.s2.angle_min = min_b_s2;
  lb.s0.angle_min = min_c_s0;
  lb.s1.angle_min = min_c_s1;
  lb.s2.angle_min = min_c_s2;
  lf.s0.angle_min = min_d_s0;
  lf.s1.angle_min = min_d_s1;
  lf.s2.angle_min = min_d_s2;
}
/****************************最大角度設定関数***************************/
void Set_Angle_Max(double max_a_s0, double max_a_s1, double max_a_s2,
                   double max_b_s0, double max_b_s1, double max_b_s2,
                   double max_c_s0, double max_c_s1, double max_c_s2,
                   double max_d_s0, double max_d_s1, double max_d_s2)
{
  rf.s0.angle_max = max_a_s0;
  rf.s1.angle_max = max_a_s1;
  rf.s2.angle_max = max_a_s2;
  rb.s0.angle_max = max_b_s0;
  rb.s1.angle_max = max_b_s1;
  rb.s2.angle_max = max_b_s2;
  lb.s0.angle_max = max_c_s0;
  lb.s1.angle_max = max_c_s1;
  lb.s2.angle_max = max_c_s2;
  lf.s0.angle_max = max_d_s0;
  lf.s1.angle_max = max_d_s1;
  lf.s2.angle_max = max_d_s2;
}

/**************************ユーザーボタン押すま待機する関数*******************/
#ifndef ODE
void Wait_UB(void)
{

  pc.printf("Please push user button.\n");

  while (UB)
  {
    wait(0.01);
  };

  while (!UB)
  {
    wait(0.01);
  };
  wait(0.5);
}
#endif

/************************胴体を上げる(クリーピング用)*************************************/
void Body_Up(double x0, double y0, double y1, double all_height)
{
  Output_Coordinate(x0, -y1, 0,
                    x0, -y1, 0,
                    x0, y0, 0,
                    x0, y0, 0,
                    0,
                    0, 1); //xo、yoで足を浮かせた状態
  wait(0.5);

  Output_Coordinate(x0, -y1, 0,
                    x0, -y1, 0,
                    x0, y0, 0,
                    x0, y0, 0,
                    all_height,
                    20, 0.05); //(1)
}

/************************胴体を上げる(大の字)*************************************/
void Body_Up_Dainozi(double x0, double y0, double all_height)
{
  Output_Coordinate(x0, y0, 0,
                    x0, y0, 0,
                    x0, y0, 0,
                    x0, y0, 0,
                    0,
                    0, 1); //xo、yoで足を浮かせた状態
  wait(0.5);

  Output_Coordinate(x0, y0, 0,
                    x0, y0, 0,
                    x0, y0, 0,
                    x0, y0, 0,
                    all_height,
                    20, 0.05); //(1)
}

/*****************************胴体を下げる*******************************/
void Body_Down(double x0, double y0)
{
  Output_Coordinate(x0, y0, 0.01,
                    x0, y0, 0.01,
                    x0, y0, 0.01,
                    x0, y0, 0.01,
                    0,
                    100, 0.05); //xo、yoで足を浮かせた状態
}

/**************************腰振り**********************************/
void Hip_Shaking(double x0, double y0, double all_height,
                 double shaking_angle, int shaking_times) //rotation_angleは回転角度[deg]
{

  Output_Angle(rf.s0.angle_old + shaking_angle, rf.s1.angle_old, rf.s2.angle_old,
               rb.s0.angle_old - shaking_angle, rb.s1.angle_old, rb.s2.angle_old,
               lb.s0.angle_old + shaking_angle, lb.s1.angle_old, lb.s2.angle_old,
               lf.s0.angle_old - shaking_angle, lf.s1.angle_old, lf.s2.angle_old,
               10, 0.1);

  for (int i(1); i <= shaking_times; i++)
  {
    Output_Angle(rf.s0.angle_old - shaking_angle * 2, rf.s1.angle_old, rf.s2.angle_old,
                 rb.s0.angle_old + shaking_angle * 2, rb.s1.angle_old, rb.s2.angle_old,
                 lb.s0.angle_old - shaking_angle * 2, lb.s1.angle_old, lb.s2.angle_old,
                 lf.s0.angle_old + shaking_angle * 2, lf.s1.angle_old, lf.s2.angle_old,
                 10, 0.1);

    Output_Angle(rf.s0.angle_old + shaking_angle * 2, rf.s1.angle_old, rf.s2.angle_old,
                 rb.s0.angle_old - shaking_angle * 2, rb.s1.angle_old, rb.s2.angle_old,
                 lb.s0.angle_old + shaking_angle * 2, lb.s1.angle_old, lb.s2.angle_old,
                 lf.s0.angle_old - shaking_angle * 2, lf.s1.angle_old, lf.s2.angle_old,
                 10, 0.1);
  }

  Output_Angle(rf.s0.angle_old - shaking_angle, rf.s1.angle_old, rf.s2.angle_old,
               rb.s0.angle_old + shaking_angle, rb.s1.angle_old, rb.s2.angle_old,
               lb.s0.angle_old - shaking_angle, lb.s1.angle_old, lb.s2.angle_old,
               lf.s0.angle_old + shaking_angle, lf.s1.angle_old, lf.s2.angle_old,
               10, 0.1);
}

/***************************オフセットの姿勢************************/
void Offset_Position(void)
{
  Output_Angle(0, 0, 0,
               0, 0, 0,
               0, 0, 0,
               0, 0, 0,
               0, 1);
  pc.printf("Offset_Position!\n");
}

/*************************足格納姿勢**************************/
void Store_Position(void)
{
  Output_Angle(45, -90, -30,
               45, -90, -30,
               45, -90, -30,
               45, -90, -30,
               0, 1);
  pc.printf("Store_Position!\n");
}

/********************ジャイロセンサのオフセット設定**********************/
void Set_Gyro_Offset(void)
{
  bno.setmode(OPERATION_MODE_NDOF); //BNO055初期化
  pitch_offset = roll_offset = yaw_offset = 0.0;
  /*
  for (int i(0); i < 20; i++)
  {
    bno.get_angles();
    pitch_offset += bno.euler.pitch / 20;
    roll_offset += bno.euler.roll / 20;
    yaw_offset += bno.euler.yaw / 20;
    wait(0.1);
  }*/
  bno.get_angles();
  roll_offset = bno.euler.roll;
  pitch_offset = bno.euler.pitch;
  yaw_offset = bno.euler.yaw;
  pc.printf("OFFSET pitch : %lf, roll : %lf, yaw : %lf\n", pitch_offset, roll_offset, yaw_offset);
}

/*********************距離センサ**********************/
double Distance(void)
{
  double distance;
  distance = (160 / 7) * (35 / (35 * distance_sensor.read() * 3.3 - 4)); //[cm]
  //pc.printf("distance= %lf\n", distance);
  return distance;
}

/*********************サーボドライバ*******************/
void setServoPulse(uint8_t n, float pulse)
{
  pwm.setPWM(n, 0, pulse);
#ifdef ODE
  setangle[(int)(n / 3)][n % 3] = (pulse * 5 - 507) * 135 / 1024;
#endif
}

void initServoDriver()
{
  pwm.begin();
  pwm.setPrescale(121);     // set 20ms for generic servos
  pwm.frequencyI2C(400000); //400kHz fast I2C comunication
}

/*********************自己位置推定********************/
void Position_estimate(double _step_length)
{
  //bno.get_angles();
  //yaw = bno.euler.yaw - yaw_offset;
  x_now += (_step_length * sin(yaw / 360)) * 0.7;
  y_now += (_step_length * cos(yaw / 360)) * 0.7;
}