#include "Common.h"
#include "Creeping.h"
#include "Output.h"
#include "Rotation.h"
#include "Function.h"
#include "SO1602A.h"
#include "PCA9685.h"
#include "Mode_select.h"
#include "Gerge.h"
#include "Gyro_sensor.h"
#include "Px4flow_calc.h"

//クリーピングのパレメータはCreeping.hにある

/**********************************************************************************/
//                                 モード切替                                      //
/**********************************************************************************/

int mode(1);

int state(0);      //リスタート用
bool yaw_mode(1);  //yaw角補正、x軸補正との共存不可
bool x_mode(0);    //x軸補正、yaw角補正との共存不可
int pc_debug(3);   //0ならprintfなし、1ならprintfあり、3ならグラフ化用の出力
int oled_debug(1); //OLEDのモード
bool field(0);     //0なら青フィールド、1なら赤フィールド

/************************固定パラメーター*******************/
const double ROPE_STEP_HEIGHT(0.80);
const double ROPE_ALL_HEIGHT(0.30);
const double BODY_HEIGHT_SAND(0.23);

/************************各種ピン設定***********************/
Serial pc(USBTX, USBRX, 115200); //PCとのシリアル通信
DigitalIn UB(USER_BUTTON);       //マイコン上のボタン
I2C i2c_1(PB_7, PB_8);
I2C i2c_2(PC_12, PB_10);
BNO055 bno(PC_12, PB_10);          //BNO055
AnalogIn hand_sensor(PC_0);        //手の検知用距離センサ
AnalogIn gerge_sensor(PC_1);       //ゲルゲの検知用距離センサ
SO1602A oled(i2c_1);               //OLEDディスプレイ
I2C optical_flow(PC_9, PA_8);      //オプティカルフローセンサ
DigitalIn button_white(PC_7);      //白ボタン
DigitalIn button_red(PA_6);        //赤ボタン
DigitalIn button_blue_left(PA_7);  //青_1ボタン(左下)
DigitalIn button_blue_right(PB_6); //青_2ボタン(右下)
DigitalIn micro_switch(PA_9);      //ゲルゲ検知用マイクロスイッチ
PwmOut gerge_1(PA_10);             //ゲルゲ掴む用
PwmOut gerge_2(PA_15);             //ゲルゲを掲げる用
PX4Flow px4flow = PX4Flow();       //PX4FLOW

#ifndef ODE
PCA9685 pwm(PB_7, PB_8); //サーボドライバ
#else
PCA9685 pwm;
#endif

/*****************グローバル変数****************/
double roll, pitch, yaw;
double roll_offset, pitch_offset, yaw_offset;
double all_height_old;
bool leg_type(0);          //0なら右足が寄ってる状態、1なら左足が寄ってる状態
double adjust_y(ADJUST_Y); //重心を前にずらす[m]
bool attitude_control(0);  //姿勢制御オフ
double yaw_ref(0.0); //Yaw角の目標値(機体の向き)
double x_ref(0.0);   //x軸方向の目標座標
double x_now(0.0);   //機体の推定x座標[m]
double y_now(0.0);   //機体の推定y座標[m]
int16_t px = 0;      // +x方向が基板上の-y方向 たぶんセンチメートル(PX4FLOW用)
int16_t py = 0;      // +y方向が基板上の+x方向 たぶんセンチメートル(PX4FLOW用)

/****************インスタンス化*********************/
Servo rf_servo[3], rb_servo[3], lb_servo[3], lf_servo[3];
Leg rf(rf_servo[0], rf_servo[1], rf_servo[2]);
Leg rb(rb_servo[0], rb_servo[1], rb_servo[2]);
Leg lb(lb_servo[0], lb_servo[1], lb_servo[2]);
Leg lf(lf_servo[0], lf_servo[1], lf_servo[2]);

/******************************************************************************/
//                                 MAIN                                       //
/******************************************************************************/
#ifdef ODE
void main2()
#else
int main()
#endif
{
  /***********初期化**************/
  pc.baud(9600);                       //シリアル通信初期設定
  initServoDriver();                   //サーボドライバ初期化
  bno.setmode(OPERATION_MODE_IMUPLUS); //BNO055初期化
  oled.init();                         //OLEDディスプレイ初期化
  Gerge_servo_init();                  //ゲルゲ受け渡し用サーボの初期化

  /***************起動**************/
  oled.printf("    Hello!! \n   KRA Ilias");
  Set_Gyro_Offset(); //ジャイロのオフセット設定
  pc.printf("Initialized\n");
  wait(0.5);
  oled.clear();

    /************モード選択*************/
#ifndef ODE
  Mode_select();
#endif

  /******************各種設定*********************/
  oled.clear();
  oled.printf("Program Start!");
  Set_Angle_Offset(81, 113, 100,
                   127, 124, 99,
                   60, 100, 100,
                   144, 157, 112);
  Set_Angle_Min(-100, -120, -120,
                -100, -120, -120,
                -100, -120, -120,
                -100, -120, -120); //最小角度設定
  Set_Angle_Max(100, 120, 120,
                100, 120, 120,
                100, 120, 120,
                100, 120, 120); //最大角度設定

  /**************************************************************************************/
  //                                   MODE 0                                           //
  /**************************************************************************************/
  if (mode == ALL_1) //全体の流れ(カクカク行くver) フィールド選択には未対応
  {
    /**************************リスタート用*********************/
    if (state != 0)
      Body_Up_LegType0(X0, Y0, Y1, ALL_HEIGHT);
    if (state == 1)
      goto ALL_1_STATE_1;
    else if (state == 2)
      goto ALL_1_STATE_2;
    else if (state == 3)
      goto ALL_1_STATE_3;
    else if (state == 4)
      goto ALL_1_STATE_4;

    /***********************************************************/
    Body_Up_LegType0(X0, Y0, Y1, 0.20);
    wait(1);
  //Wait_gerge_setting(); //ゲルゲセッティング
  //Receive_gerge();      //ゲルゲ受け渡し

  /**************ウルトゥー1出発***************/
  ALL_1_STATE_1:
    state = 1;
    Output_Coordinate(X0, Y0, 0,
                      X0, Y0, 0,
                      X0, Y0, 0,
                      X0, Y0, 0,
                      ALL_HEIGHT,
                      10, 0.01);
    do
    {
      if (y_now < 2.8 - STEP_LENGTH * 2 * 0.5) //2歩分以上手前
      {
        Creeping_repeat_2step(X0, Y0, Y1, ALL_HEIGHT, ADJUST_Y,
                              STEP_LENGTH, STEP_HEIGHT,
                              CREEPING_STEP_TIMES, CREEPING_WAIT_TIME);
      }
      else //1歩分以上2歩分未満手前
      {
        Creeping_repeat_1step(X0, Y0, Y1, ALL_HEIGHT, ADJUST_Y,
                              STEP_LENGTH, STEP_HEIGHT,
                              CREEPING_STEP_TIMES, CREEPING_WAIT_TIME);
        leg_type = 1;
        LegType1_to_Dainozi();
        goto ALL_1_BEFORE_SAND;
      }
    } while (y_now < 2.8 - STEP_LENGTH * 0.5); //1歩分以上手前
    Output_Coordinate(X0, -Y1, rf.adjust,
                      X0, -Y1, rb.adjust,
                      X0, -Y1 + STEP_LENGTH / 2, lb.adjust,
                      X0, -Y1 + STEP_LENGTH / 2, lf.adjust,
                      ALL_HEIGHT,
                      CREEPING_STEP_TIMES * 4, CREEPING_WAIT_TIME); //(2)
    leg_type = 0;
    LegType0_to_Dainozi();
  ALL_1_BEFORE_SAND:                     //(0,3.0)に到着
    Rotation_Deg(X0, Y0, 0.30, 16.0, 3); //右回り45度回転
    Dainozi_to_LegType0();

  /**************砂丘始まり****************/
  ALL_1_STATE_2:
    state = 2;
    pc.printf("Start Sand\n");
    Get_gyro();
    yaw_ref = 45.0;

    Creeping(X0, Y0, Y1, BODY_HEIGHT_SAND, ADJUST_Y,
             STEP_LENGTH, STEP_HEIGHT,
             CREEPING_STEP_TIMES, CREEPING_WAIT_TIME,
             6); //タイマー制御

    leg_type = 0;
    LegType0_to_Dainozi();
    x_now = 0.935; //Sandで推定座標が狂うから補正をかける
    y_now = 4.5;
    /***************砂丘終わり****************/
    pc.printf("Finish Sand\n");
    Rotation_Deg(X0, Y0, 0.30, 13.0, 3); //右回り45度回転
    Dainozi_to_LegType0();
    Get_gyro();
    yaw_ref = 90.0;
    do
    {
      if (x_now < 2.225 * 0.70 - STEP_LENGTH * 2 * 0.5) //2歩分以上手前
      {
        Creeping_repeat_2step(X0, Y0, Y1, ALL_HEIGHT, ADJUST_Y,
                              STEP_LENGTH, STEP_HEIGHT,
                              CREEPING_STEP_TIMES, CREEPING_WAIT_TIME);
      }
      else //1歩分以上2歩分未満手前
      {
        Creeping_repeat_1step(X0, Y0, Y1, ALL_HEIGHT, ADJUST_Y,
                              STEP_LENGTH, STEP_HEIGHT,
                              CREEPING_STEP_TIMES, CREEPING_WAIT_TIME);
        leg_type = 1;
        LegType1_to_Dainozi();
        goto ALL_1_BEFORE_ROPE;
      }
    } while (x_now < 2.225 * 0.70 - STEP_LENGTH * 0.5); //1歩分以上手前
    Output_Coordinate(X0, -Y1, rf.adjust,
                      X0, -Y1, rb.adjust,
                      X0, -Y1 + STEP_LENGTH / 2, lb.adjust,
                      X0, -Y1 + STEP_LENGTH / 2, lf.adjust,
                      ALL_HEIGHT,
                      CREEPING_STEP_TIMES * 4, CREEPING_WAIT_TIME); //(2)
    leg_type = 0;
    LegType0_to_Dainozi();

  ALL_1_BEFORE_ROPE:
    Rotation_Deg(X0, Y0, 0.30, -15.0, 6); //左回り90度回転
    Dainozi_to_LegType0();
    Get_gyro();
    yaw_ref = 0.0;

  /***************ロープ始まり**************/
  ALL_1_STATE_3:
    state = 3;
    pc.printf("Start Rope\n");
    yaw_ref = 0.0;
    do
    {
      if (y_now < 7.0 * 1.1 - STEP_LENGTH * 2 * 0.5) //2歩分以上手前
      {
        Creeping_repeat_2step(X0, Y0, Y1, 0.30, ADJUST_Y,
                              STEP_LENGTH, 0.30,
                              CREEPING_STEP_TIMES, CREEPING_WAIT_TIME);
      }
      else //1歩分以上2歩分未満手前
      {
        Creeping_repeat_1step(X0, Y0, Y1, 0.30, ADJUST_Y,
                              STEP_LENGTH, 0.30,
                              CREEPING_STEP_TIMES, CREEPING_WAIT_TIME);
        leg_type = 1;
        Output_Coordinate(0.17, -0.040 - adjust_y, 0,
                          0.17, -0.040 + adjust_y, 0,
                          0.17, 0.15 + adjust_y, 0,
                          0.17, 0.15 - adjust_y, 0,
                          0.25,
                          0, 1); //LegType0
        goto ALL_1_STATE_4;
      }
    } while (y_now < 7.0 * 1.1 - STEP_LENGTH * 0.5); //1歩分以上手前
    Output_Coordinate(X0, -Y1, rf.adjust,
                      X0, -Y1, rb.adjust,
                      X0, -Y1 + STEP_LENGTH / 2, lb.adjust,
                      X0, -Y1 + STEP_LENGTH / 2, lf.adjust,
                      0.25,
                      CREEPING_STEP_TIMES * 4, CREEPING_WAIT_TIME); //(2)
    leg_type = 0;

  /**************ウルトゥー2到着***************/
  ALL_1_STATE_4:
    state = 4;
    oled.clear();
    oled.printf("Arrive at Urutu2");
    wait(1);

#ifndef ODE
    while (Hand_distance() > 20 || Hand_distance() < 10)
    {
      oled.clear();
      oled.printf("Please hold Hand");
      wait(0.5);
    }
#endif
    /*****************坂登り開始*****************/
    oled.clear();
    oled.printf("Go to Slope\n");
    yaw_ref = -90.0;
    rf.adjust = 0.017;
    lf.adjust = 0.017;
    Creeping(X0, Y0, Y1, 0.25, 0.1,
             0.33, 0.25,
             5, 0.002,
             13);

    oled.clear();
    oled.printf("Finish climbing");

    /********************坂登り終了*******************/

    Output_Coordinate(0.15, 0.15 - adjust_y, 0,
                      0.15, 0.15 + adjust_y, 0,
                      0.15, 0.15 + adjust_y, 0,
                      0.15, 0.15 - adjust_y, 0,
                      0.30,
                      20, 0.05);

    Raise_gerge();

    oled.clear();
    oled.printf("Finish!!");
  }
  /**************************************************************************************/
  //                                   MODE 1                                           //
  /**************************************************************************************/
  else if (mode == ALL_2) //全体の流れ(なめらかにver)
  {
    /**************************リスタート用*********************/
    if (state != 0)
      Body_Up_LegType0(X0, Y0, Y1, ALL_HEIGHT);
    if (state == 1)
      goto ALL_2_STATE_1;
    else if (state == 2)
      goto ALL_2_STATE_2;
    else if (state == 3)
      goto ALL_2_STATE_3;
    else if (state == 4)
      goto ALL_2_STATE_4;

    /***********************************************************/
    Body_Up_LegType0(X0, Y0, Y1, 0.20);
    wait(1);
    Wait_gerge_setting(); //ゲルゲセッティング
    Receive_gerge();      //ゲルゲ受け渡し

    /**************ウルトゥー1出発***************/
  ALL_2_STATE_1:
    state = 1;
    Output_Coordinate(X0, Y0, 0,
                      X0, Y0, 0,
                      X0, Y0, 0,
                      X0, Y0, 0,
                      ALL_HEIGHT,
                      10, 0.01);
    do
    {
      if (y_now < 3.0 - STEP_LENGTH * 2 * 0.5) //2歩分以上手前
      {
        Creeping_repeat_2step(X0, Y0, Y1, ALL_HEIGHT, ADJUST_Y,
                              STEP_LENGTH, STEP_HEIGHT,
                              CREEPING_STEP_TIMES, CREEPING_WAIT_TIME);
      }
      else //1歩分以上2歩分未満手前
      {
        Creeping_repeat_1step(X0, Y0, Y1, ALL_HEIGHT, ADJUST_Y,
                              STEP_LENGTH, STEP_HEIGHT,
                              CREEPING_STEP_TIMES, CREEPING_WAIT_TIME);
        leg_type = 1;
        LegType1_to_Dainozi();
        goto ALL_2_BEFORE_SAND;
      }
    } while (y_now < 3.0 - STEP_LENGTH * 0.5); //1歩分以上手前
    Output_Coordinate(X0, -Y1, rf.adjust,
                      X0, -Y1, rb.adjust,
                      X0, -Y1 + STEP_LENGTH / 2, lb.adjust,
                      X0, -Y1 + STEP_LENGTH / 2, lf.adjust,
                      ALL_HEIGHT,
                      CREEPING_STEP_TIMES * 4, CREEPING_WAIT_TIME); //(2)
    leg_type = 0;
    LegType0_to_Dainozi();
  ALL_2_BEFORE_SAND: //(0,3.0)に到着
    if (field == 0)
      Rotation_Deg(X0, Y0, 0.30, 14.0, 3); //右回り45度回転
    else
      Rotation_Deg(X0, Y0, 0.30, -14.0, 3); //左回り45度回転
    Dainozi_to_LegType0();

  /**************砂丘始まり****************/
  ALL_2_STATE_2:
    state = 2;
    pc.printf("Start Sand\n");
    Get_gyro();
    if (field == 0)
      yaw_ref = 45.0;
    else
      yaw_ref = -45.0;

    Creeping(X0, Y0, Y1, BODY_HEIGHT_SAND, ADJUST_Y,
             STEP_LENGTH, STEP_HEIGHT + 0.1,
             CREEPING_STEP_TIMES, CREEPING_WAIT_TIME,
             7); //タイマー制御(砂丘だけならSTEP_TIMESは6)

    leg_type = 0;
    LegType0_to_Dainozi();
    if (field == 0)
      Rotation_Deg(X0, Y0, 0.30, -15.0, 6); //左回り90度回転
    else
      Rotation_Deg(X0, Y0, 0.30, 15.0, 6); //右回り90度回転

    Dainozi_to_LegType0();
    yaw_ref = 0.0;

  /***************ロープ始まり**************/
  ALL_2_STATE_3:
    state = 3;
    pc.printf("Start Rope\n");
    yaw_ref = 0.0;
    do
    {
      if (y_now < 7.0 * 1.1 - STEP_LENGTH * 2 * 0.5) //2歩分以上手前
      {
        Creeping_repeat_2step(X0, Y0, Y1, ROPE_ALL_HEIGHT, ADJUST_Y,
                              STEP_LENGTH, ROPE_STEP_HEIGHT,
                              CREEPING_STEP_TIMES, CREEPING_WAIT_TIME);
      }
      else //1歩分以上2歩分未満手前
      {
        Creeping_repeat_1step(X0, Y0, Y1, ROPE_ALL_HEIGHT, ADJUST_Y,
                              STEP_LENGTH, ROPE_STEP_HEIGHT,
                              CREEPING_STEP_TIMES, CREEPING_WAIT_TIME);
        leg_type = 1;
        Output_Coordinate(0.17, -0.040 - adjust_y, 0,
                          0.17, -0.040 + adjust_y, 0,
                          0.17, 0.15 + adjust_y, 0,
                          0.17, 0.15 - adjust_y, 0,
                          0.25,
                          0, 1); //LegType0
        goto ALL_2_STATE_4;
      }
    } while (y_now < 7.0 * 1.1 - STEP_LENGTH * 0.5); //1歩分以上手前
    Output_Coordinate(X0, -Y1, rf.adjust,
                      X0, -Y1, rb.adjust,
                      X0, -Y1 + STEP_LENGTH / 2, lb.adjust,
                      X0, -Y1 + STEP_LENGTH / 2, lf.adjust,
                      0.25,
                      CREEPING_STEP_TIMES * 4, CREEPING_WAIT_TIME); //(2)
    leg_type = 0;

  /**************ウルトゥー2到着***************/
  ALL_2_STATE_4:
    state = 4;
    oled.clear();
    oled.printf("Arrive at Urutu2");
    wait(1);

#ifndef ODE
    while (Hand_distance() > 20 || Hand_distance() < 10)
    {
      oled.clear();
      oled.printf("Please hold Hand");
      wait(0.5);
    }
#endif
    /*****************坂登り開始*****************/
    oled.clear();
    oled.printf("Go to Slope\n");
    if (field == 0)
      yaw_ref = -90.0;
    else
      yaw_ref = 90.0;

    rf.adjust = 0.017;
    lf.adjust = 0.017;
    Creeping(X0, Y0, Y1, 0.25, 0.1,
             0.33, 0.25,
             5, 0.002,
             13);

    oled.clear();
    oled.printf("Finish climbing");

    /********************坂登り終了*******************/

    Output_Coordinate(0.15, 0.15 - adjust_y, 0,
                      0.15, 0.15 + adjust_y, 0,
                      0.15, 0.15 + adjust_y, 0,
                      0.15, 0.15 - adjust_y, 0,
                      0.30,
                      20, 0.05);

    Raise_gerge();

    oled.clear();
    oled.printf("Finish!!");
  }
  /**************************************************************************************/
  //                                   MODE 2                                           //
  /**************************************************************************************/
  else if (mode == WALKING) //普通の歩行
  {
    Wait_gerge_setting();
    Receive_gerge();
    x_mode = 0;
    yaw_mode = 1;
    yaw_ref = 0.0;
    x_ref = 0.0;
    Body_Up_LegType0(X0, Y0, Y1, ALL_HEIGHT);
    Creeping(X0, Y0, Y1, ALL_HEIGHT, 0.03,
             STEP_LENGTH, STEP_HEIGHT,
             CREEPING_STEP_TIMES, CREEPING_WAIT_TIME,
             100);
  }
  /**************************************************************************************/
  //                                   MODE 3                                           //
  /**************************************************************************************/
  else if (mode == STAND)
  { // ずっと立っているだけのモード
    Output_Coordinate(0.15, 0.15 - adjust_y, 0.3,
                      0.15, 0.15 + adjust_y, 0.3,
                      0.15, 0.15 + adjust_y, 0.3,
                      0.15, 0.15 - adjust_y, 0.3,
                      0.00,
                      0, 0.05);
    Output_Coordinate(0.15, 0.15 - adjust_y, 0,
                      0.15, 0.15 + adjust_y, 0,
                      0.15, 0.15 + adjust_y, 0,
                      0.15, 0.15 - adjust_y, 0,
                      0.20,
                      20, 0.05);
    wait(1);

    Set_Gyro_Offset(); //ジャイロのオフセット設定
    while (1)
    {
      Output_Coordinate(0.15, 0.15 - adjust_y, 0,
                        0.15, 0.15 + adjust_y, 0,
                        0.15, 0.15 + adjust_y, 0,
                        0.15, 0.15 - adjust_y, 0,
                        0.20,
                        20, 0.05);
    }
  }
  /**************************************************************************************/
  //                                   MODE 4                                           //
  /**************************************************************************************/
  else if (mode == SENSOR)
  { //赤外線センサの値表示
    while (1)
    {
      oled.clear();
      oled.printf("gerge:%2.2lf\n", gerge_sensor.read());
      oled.printf("hand:%2.2lf", hand_sensor.read());
      wait(0.1);
    }
  }
  /**************************************************************************************/
  //                                   MODE 5                                           //
  /**************************************************************************************/
  else if (mode == PX4FLOW)
  { //PX4FLOW
    while (1)
    {
      Get_px4flow();
      oled.clear();
      oled.printf("(px,py)=(%d, %d)", px, py);
      pc.printf("(px,py)=(%d, %d)\n", px, py);
      wait(0.5);
    }
  }
  /**************************************************************************************/
  //                                   MODE 6                                           //
  /**************************************************************************************/
  else if (mode == OFFSET)
  { //オフセット
    Offset_Position();
    Wait_UB();
  }

  while (button_white == 0)
    wait(0.1);

  Body_Down(X0, Y0);

  while (button_white == 0)
    wait(0.1);

  Store_Position();

#ifndef ODE
  return 0;
#endif
}