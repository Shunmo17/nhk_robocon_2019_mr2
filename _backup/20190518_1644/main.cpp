#include "Common.h"
#include "Creeping.h"
#include "Output.h"
#include "Rotation.h"
#include "Trot.h"
#include "Function.h"
#include "SO1602A.h"
#include "PCA9685.h"
#include "VL53L0X.h"
#include "Mode_select.h"
#include "Distance_sensor.h"
#include "Gerge.h"
#include "Gyro_sensor.h"
#include "Px4flow_calc.h"

//クリーピングのパレメータはCreeping.hにある//

/************************各種ピン設定***********************/
Serial pc(USBTX, USBRX, 115200); //PCとのシリアル通信
DigitalIn UB(USER_BUTTON);       //マイコン上のボタン
I2C i2c_1(PB_7, PB_8);
I2C i2c_2(PC_12, PB_10);
BNO055 bno(PC_12, PB_10);       //BNO055
AnalogIn distance_sensor(PC_0); //距離センサ
SO1602A oled(i2c_1);            //OLEDディスプレイ
I2C optical_flow(PC_9, PA_8);   //オプティカルフローセンサ

Timer dt;

#ifndef ODE
PCA9685 pwm(PB_7, PB_8); //サーボドライバ
#else
PCA9685 pwm;
#endif

Timer timer_vl[2]; //VL53L0x
VL53L0X vl[2] = {VL53L0X(&i2c_1, &timer_vl[0]), VL53L0X(&i2c_1, &timer_vl[1])};
DigitalInOut Xshut[2] = {DigitalInOut(PC_8), DigitalInOut(PC_6)};

PX4Flow px4flow = PX4Flow(); //PX4FLOW

DigitalIn button_white(PC_7);      //白ボタン
DigitalIn button_red(PA_6);        //赤ボタン
DigitalIn button_blue_left(PA_7);  //青_1ボタン(左下)
DigitalIn button_blue_right(PB_6); //青_2ボタン(右下)

//PwmOut speaker(PC_7); //スピーカー

DigitalIn micro_switch(PA_9); //マイクロスイッチ
PwmOut gerge_1(PA_10);        //ゲルゲ掴む用
PwmOut gerge_2(PA_15);        //ゲルゲを掲げる用

/*****************グローバル変数****************/
double roll, pitch, yaw;
double roll_offset, pitch_offset, yaw_offset;
double all_height_old;
int mode(4);            //0なら歩行するだけ、1なら全体の流れ、2ならSlope、3なら立つだけ、4なら回転だけ、5はテストモード
int state(0);           //リスタート用
bool yaw_mode(0);       //yaw角補正、x軸補正との共存不可
bool x_mode(0);         //x軸補正、yaw角補正との共存不可
bool lidarlite_mode(1); //lidarliteでの測距
int pc_debug(1);        //0ならprintfなし、1ならprintfあり、3ならグラフ化用の出力
int oled_debug(3);
bool leg_type(0);          //0なら右足が寄ってる状態、1なら左足が寄ってる状態
double adjust_y(ADJUST_Y); //重心を前にずらす[m]
bool attitude_control(0);  //姿勢制御オフ
double body_height_sand(0.23);

double yaw_ref(0.0); //Yaw角の目標値(機体の向き)
double x_ref(0.0);   //x軸方向の目標座標
double x_now(0.0);   //機体の推定x座標[m]
double y_now(0.0);   //機体の推定y座標[m]

int16_t px = 0; // +x方向が基板上の-y方向 たぶんセンチメートル(PX4FLOW用)
int16_t py = 0; // +y方向が基板上の+x方向 たぶんセンチメートル(PX4FLOW用)

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
  //lidar.configure();                //Lidarlite初期化

  oled.init();        //OLEDディスプレイ初期化
  Gerge_servo_init(); //ゲルゲ受け渡し用サーボの初期化

  /***************起動**************/
  oled.printf("    Hello!! \n   KRA Ilias");
  pc.printf("Initialized\n");
  wait(0.5);

  /************モード選択*************/
#ifndef ODE
  Mode_select();
#endif

  /******************各種設定*********************/
  oled.clear();
  oled.printf("Program Start!");

  Set_Angle_Offset(85, 113, 100,
                   122, 124, 99,
                   65, 100, 100,
                   139, 157, 112);

  Set_Angle_Min(-100, -120, -120,
                -100, -120, -120,
                -100, -120, -120,
                -100, -120, -120); //最小角度設定

  Set_Angle_Max(100, 120, 120,
                100, 120, 120,
                100, 120, 120,
                100, 120, 120); //最大角度設定

  Set_Gyro_Offset(); //ジャイロのオフセット設定

  //Offset_Position();
  //Wait_UB();
  while(1){
    pc.printf("%2.2lf\n",Hand_distance());
    wait(0.5);
  }

  /***********************************************************/
  /**************************************************************************************/
  //                                   MODE 0                                           //
  /**************************************************************************************/
  if (mode == 0) //普通の歩行
  {
    x_mode = 1;
    yaw_mode = 0;
    yaw_ref = 0.0;
    x_ref = 0.0;

    Body_Up_LegType0(X0, Y0, Y1, ALL_HEIGHT);

    Creeping(X0, Y0, Y1, ALL_HEIGHT, 0.1,
             STEP_LENGTH, STEP_HEIGHT,
             CREEPING_STEP_TIMES, CREEPING_WAIT_TIME,
             100);
  }
  /**************************************************************************************/
  //                                   MODE 1                                           //
  /**************************************************************************************/
  else if (mode == 1) //全体の流れ
  {
    /**************************リスタート用*********************/
    if (state != 0)
      Body_Up_LegType0(X0, Y0, Y1, ALL_HEIGHT);
    if (state == 1)
      goto STATE_1;
    else if (state == 2)
      goto STATE_2;
    else if (state == 3)
      goto STATE_3;
    else if (state == 4)
      goto STATE_4;

    /***********************************************************/
    Body_Up_LegType0(X0, Y0, Y1, 0.20);
    wait(1);
    oled.clear();
    oled.printf("Please setting");
    //Wait_gerge_setting();

    /**************ゲルゲ受け渡し*************/
    /*
    while (micro_switch == 0)
    {
      oled.clear();
      oled.printf("Waiting Gerge");
      wait(0.1);
    }
    Hold_gerge();
    oled.clear();
    oled.printf("Get Gerge!");

    wait(1.0);

    */
    /**************ウルトゥー1出発***************/
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
        goto LABEL_1;
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

  LABEL_1:                               //(0,3.0)に到着
    Rotation_Deg(X0, Y0, 0.30, 16.0, 3); //右回り45度回転
    Dainozi_to_LegType0();

    /**************砂丘始まり****************/
    pc.printf("Start Sand\n");
    Get_gyro();
    yaw_ref = 45.0;
    Creeping(X0, Y0, Y1, body_height_sand, ADJUST_Y,
             STEP_LENGTH, STEP_HEIGHT,
             CREEPING_STEP_TIMES, CREEPING_WAIT_TIME,
             6); //タイマー制御
    /*do
    {
      if (x_now < 0.935 - STEP_LENGTH * 2 * (sqrt(2) / 2) * 0.5) //2歩分以上手前
      {
        Creeping_repeat_2step(X0, Y0, Y1, body_height_sand, ADJUST_Y,
                              STEP_LENGTH, STEP_HEIGHT,
                              CREEPING_STEP_TIMES, CREEPING_WAIT_TIME);
      }
      else //1歩分以上2歩分未満手前
      {
        Creeping_repeat_1step(X0, Y0, Y1, body_height_sand, ADJUST_Y,
                              STEP_LENGTH, STEP_HEIGHT,
                              CREEPING_STEP_TIMES, CREEPING_WAIT_TIME);
        leg_type = 1;
        LegType1_to_Dainozi();
        goto LABEL_2;
      }
    } while (x_now < 0.935 - STEP_LENGTH * (sqrt(2) / 2) * 0.5); //1歩分以上手前
    Output_Coordinate(X0, -Y1, rf.adjust,
                      X0, -Y1, rb.adjust,
                      X0, -Y1 + STEP_LENGTH / 2, lb.adjust,
                      X0, -Y1 + STEP_LENGTH / 2, lf.adjust,
                      ALL_HEIGHT,
                      CREEPING_STEP_TIMES * 4, CREEPING_WAIT_TIME); //(2)
                      */
    leg_type = 0;
    LegType0_to_Dainozi();
    x_now = 0.935; //Sandで推定座標が狂うから補正をかける
    y_now = 4.5;
    /***************砂丘終わり****************/

    //LABEL_2:
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
        goto LABEL_3;
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

  LABEL_3:
    Rotation_Deg(X0, Y0, 0.30, -15.0, 6); //左回り90度回転
    Dainozi_to_LegType0();
    Get_gyro();
    yaw_ref = 0.0;

    /***************ロープ始まり**************/
    pc.printf("Start Rope\n");
    yaw_ref = 0.0;
    do
    {
      if (y_now < 7.0 * 1.1 - STEP_LENGTH * 2 * 0.5) //2歩分以上手前
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
        Output_Coordinate(0.17, -0.040 - adjust_y, 0,
                          0.17, -0.040 + adjust_y, 0,
                          0.17, 0.15 + adjust_y, 0,
                          0.17, 0.15 - adjust_y, 0,
                          0.28,
                          0, 1); //LegType0
        goto LABEL_4;
      }
    } while (y_now < 7.0 * 1.1 - STEP_LENGTH * 0.5); //1歩分以上手前
    Output_Coordinate(X0, -Y1, rf.adjust,
                      X0, -Y1, rb.adjust,
                      X0, -Y1 + STEP_LENGTH / 2, lb.adjust,
                      X0, -Y1 + STEP_LENGTH / 2, lf.adjust,
                      ALL_HEIGHT,
                      CREEPING_STEP_TIMES * 4, CREEPING_WAIT_TIME); //(2)
    leg_type = 0;

    /**************ウルトゥー2到着***************/
  LABEL_4:
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

    rf.adjust = 0.015;
    lf.adjust = 0.015;
    Creeping(X0, Y0, Y1, 0.25, 0.07,
             0.35, 0.25,
             5, 0.002,
             9);

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
  //                                   MODE 3                                           //
  /**************************************************************************************/
  else if (mode == 3)
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
  else if (mode == 4)
  { //リスタート前提のタイマー制御
    yaw_mode = 0;
    ratio_right = 0.87;
    /*************************リスタート用*********************/
    if (state != 0)
    {
      Body_Up_LegType0(X0, Y0, Y1, ALL_HEIGHT);
      gerge_2.pulsewidth_us((int)(1000 * ((145 / 180) * 2.4 + 0.5)));
    }
    if (state == 1)
      goto STATE_1;
    else if (state == 2)
      goto STATE_2;
    else if (state == 3)
      goto STATE_3;
    else if (state == 4)
      goto STATE_4;

    /***********************************************************/
    Body_Up_LegType0(X0, Y0, Y1, 0.17);
    wait(1);
    oled.clear();
    oled.printf("Please setting");
    Wait_gerge_setting();

    /**************ゲルゲ受け渡し*************/
    while (micro_switch == 0)
    {
      oled.clear();
      oled.printf("Waiting Gerge");
      wait(0.1);
    }
    Hold_gerge();
    oled.clear();
    oled.printf("Get Gerge!");

    wait(2.0);

    /**************ウルトゥー1出発***************/
  STATE_1:
    Output_Coordinate(X0, Y0, 0,
                      X0, Y0, 0,
                      X0, Y0, 0,
                      X0, Y0, 0,
                      ALL_HEIGHT,
                      10, 0.01);
    Creeping(X0, Y0, Y1, ALL_HEIGHT, ADJUST_Y,
             0.39, STEP_HEIGHT,
             CREEPING_STEP_TIMES, CREEPING_WAIT_TIME,
             8);
    /*
    do
    {
      if (y_now < 3.2 - STEP_LENGTH * 2 * 0.5) //2歩分以上手前
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
        goto LABEL_4_1;
      }
    } while (y_now < 3.2 - STEP_LENGTH * 0.5); //1歩分以上手前
    Output_Coordinate(X0, -Y1, rf.adjust,
                      X0, -Y1, rb.adjust,
                      X0, -Y1 + STEP_LENGTH / 2, lb.adjust,
                      X0, -Y1 + STEP_LENGTH / 2, lf.adjust,
                      ALL_HEIGHT,
                      CREEPING_STEP_TIMES * 4, CREEPING_WAIT_TIME); //(2)
    leg_type = 0;
    */
    LegType0_to_Dainozi();
    //LABEL_4_1:
    //(0,3.0)に到着
    Rotation_Deg(X0, Y0, 0.30, 16.5, 3); //右回り45度回転
    Dainozi_to_LegType0();
    /**************砂丘始まり****************/
  STATE_2:
    state = 2;
    ratio_right = 0.8;
    ratio_left = 1.0;
    pc.printf("Start Sand\n");
    Get_gyro();
    Creeping(X0, Y0, Y1, 0.245, ADJUST_Y,
             0.39, 0.30,
             CREEPING_STEP_TIMES, CREEPING_WAIT_TIME,
             17); //タイマー制御
                  //Sandを超えたらリスタート
    ratio_right = 1.0;

  STATE_3:
    /***************ロープ始まり**************/
    state = 3;
    yaw_mode = 0;
    pc.printf("Start Rope\n");

    Creeping(X0, Y0, Y1, 0.23, ADJUST_Y,
             STEP_LENGTH, 0.30,
             CREEPING_STEP_TIMES, CREEPING_WAIT_TIME,
             15);

    /************ウルトゥー2到着***************/
  STATE_4:
    state = 4;
    oled.clear();
    oled.printf("Arrive at Urutu2");
    wait(1);

#ifndef ODE
    double distance_temp;
    do
    {
      distance_temp = Hand_distance();
      oled.clear();
      oled.printf("Please hold Hand");
      wait(0.2);
    } while (distance_temp > 20 || distance_temp < 10);
#endif
    /*****************坂登り開始*****************/
    oled.clear();
    oled.printf("Go to Slope\n");

    rf.adjust = 0.04;
    lf.adjust = 0.04;
    ratio_right = 0.93;
    yaw_mode = 1;
    Get_gyro();
    yaw_ref = yaw;
    Creeping(X0, Y0, Y1, 0.24, 0.06,
             0.35, 0.23,
             5, 0.005,
             8);

    oled.clear();
    oled.printf("Finish climbing");

    /********************坂登り終了*******************/

    Output_Coordinate(0.15, 0.15 - adjust_y, 0,
                      0.15, 0.15 + adjust_y, 0,
                      0.15, 0.15 + adjust_y, 0,
                      0.15, 0.15 - adjust_y, 0,
                      0.30,
                      10, 0.05);

    Raise_gerge();
    Rotation_Deg(X0, Y0, 0.30, 5.0, 1);

    oled.clear();
    oled.printf("Finish!!");
  }
  /**************************************************************************************/
  //                                   MODE 5                                           //
  /**************************************************************************************/
  else if (mode == 5)
  {
    yaw_ref = yaw;
    rf.adjust = 0.01;
    lf.adjust = 0.01;
    Body_Up_LegType0(X0, Y0, Y1, 0.25);
    Creeping(X0, Y0, Y1, 0.20, 0.07,
             0.35, 0.25,
             4, 0.0025,
             10);
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