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
#include "LidarLitev2.h"
#include "Gerge.h"

/************************各種ピン設定***********************/
Serial pc(USBTX, USBRX, 115200); //PCとのシリアル通信
DigitalIn UB(USER_BUTTON);       //マイコン上のボタン
I2C i2c_1(PB_7, PB_8);
I2C i2c_2(PC_12,PB_10);
BNO055 bno(PC_12, PB_10);       //BNO055
AnalogIn distance_sensor(PC_0); //距離センサ
SO1602A oled(i2c_1);            //OLEDディスプレイ
LidarLitev2 lidar(PB_7, PB_8);

Timer dt;

#ifndef ODE
PCA9685 pwm(PB_7, PB_8); //サーボドライバ
#else
PCA9685 pwm;
#endif

Timer timer_vl[2]; //VL53L0x
VL53L0X vl[2] = {VL53L0X(&i2c_1, &timer_vl[0]), VL53L0X(&i2c_1, &timer_vl[1])};
DigitalInOut Xshut[2] = {DigitalInOut(PC_8), DigitalInOut(PC_6)};

DigitalIn button_white(PC_7);      //白ボタン
DigitalIn button_red(PA_6);        //赤ボタン
DigitalIn button_blue_left(PA_7);  //青_1ボタン(左下)
DigitalIn button_blue_right(PB_6); //青_2ボタン(右下)

//PwmOut speaker(PC_7); //スピーカー

DigitalIn micro_switch(PA_8); //マイクロスイッチ
PwmOut gerge_1(PA_10);        //ゲルゲ掴む用
PwmOut gerge_2(PA_15);        //ゲルゲを掲げる用

/*****************グローバル変数****************/
double roll, pitch, yaw;
double roll_offset, pitch_offset, yaw_offset;
double all_height_old;
int mode(1);            //0なら歩行するだけ、1なら全体の流れ、2ならSlope、3なら立つだけ
bool yaw_mode(1);       //0ならyaw角補正なし、1ならyaw角補正あり
bool lidarlite_mode(1); //lidarliteで測距する場合は1
int pc_debug(1);        //0ならprintfなし、1ならprintfあり、3ならグラフ化用の出力
int oled_debug(3);
bool field(1); //0なら赤、1なら青

int vl_output[2];      //[mm]
double vl_distance[2]; //[m]
double yaw_ref(0.0);   //Yaw角の目標値(機体の向き)
double delta_yaw(0.0);
double x_now(0.0); //機体のx座標[m]
double y_now(0.0); //機体のy座標[m]

/****************インスタンス化*********************/
Servo rf_servo[3], rb_servo[3], lb_servo[3], lf_servo[3];

Leg rf(rf_servo[0], rf_servo[1], rf_servo[2]);
Leg rb(rb_servo[0], rb_servo[1], rb_servo[2]);
Leg lb(lb_servo[0], lb_servo[1], lb_servo[2]);
Leg lf(lf_servo[0], lf_servo[1], lf_servo[2]);

/*************************************メイン関数***************************************/
#ifdef ODE
void main2()
#else
int main()
#endif
{
  /***********初期化**************/
  pc.baud(9600);                    //シリアル通信初期設定
  initServoDriver();                //サーボドライバ初期化
  bno.setmode(OPERATION_MODE_NDOF); //BNO055初期化
  //lidar.configure();                //Lidarlite初期化
  //VL53L0X_init(); //測距センサ初期化

  oled.init();            //OLEDディスプレイ初期化
  //i2c_1.frequency(40000); //I2C 40kHz
  //i2c_2.frequency(400000);
  Gerge_servo_init();     //ゲルゲ受け渡し用サーボの初期化

  /***************起動**************/
  oled.printf("    Hello!! \n   KRA Ilias");
  pc.printf("Initialized\n");
  wait(1.0);

  /************モード選択*************/
#ifndef ODE
  Mode_select();
#endif

  /******************各種設定*********************/
  oled.clear();
  oled.printf("Program Start!");

  Set_Angle_Offset(82, 117, 108,
                   133, 127, 102,
                   61, 102, 105,
                   153, 135, 125);

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

  /***********************************************************/

  if (mode == 0) //普通の歩行
  {
    Body_Up(0.17, 0.15, 0.040, 0.28);

    wait(1);

    if (pc_debug == 1)
      pc.printf("Walking Start!\n");

    Creeping(0.17, 0.15, 0.030, 0.29,
             0.40, 0.20,
             5, 0.01,
             15);
  }
  else if (mode == 1) //全体の流れ
  {
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

    wait(1);

    /**************ウルトゥー1出発***************/

    Body_Up(0.17, 0.15, 0.040, 0.28);

    wait(1);

    Creeping(0.17, 0.15, 0.030, 0.28,
             0.40, 0.20,
             5, 0.01,
             15);

    /**************ウルトゥー2到着***************/

    Output_Coordinate(0.17, -0.040, 0,
                      0.17, -0.040, 0,
                      0.17, 0.15, 0,
                      0.17, 0.15, 0,
                      0.28,
                      0, 1);

    oled.clear();
    oled.printf("Arrive at Urutu2");

#ifndef ODE
    while (Hand_distance() > 20 || Hand_distance() < 10)
    {
      oled.clear();
      oled.printf("Hold Hand");
      wait(0.1);
    }
#endif
    /*****************坂登り開始*****************/
    oled.clear();
    oled.printf("Go to Slope\n");

    stage = 5;
    mode = 4;

    Creeping(0.17, 0.20, 0.020, 0.28,
             0.40, 0.20,
             7, 0.01,
             7);

    oled.clear();
    oled.printf("Finish climbing");

    /********************坂登り終了*******************/

    Output_Coordinate(0.15, 0.15, 0,
                      0.15, 0.15, 0,
                      0.15, 0.15, 0,
                      0.15, 0.15, 0,
                      0.30,
                      20, 0.05);

    Rotation_Deg(0.15, 0.15, 0.30,
                 45.0, 5);

    Raise_gerge();

    oled.clear();
    oled.printf("Finish!!");
  }
  else if (mode == 3)
  { // ずっと立っているだけのモード
    Output_Coordinate(0.15, 0.15, 0.3,
                      0.15, 0.15, 0.3,
                      0.15, 0.15, 0.3,
                      0.15, 0.15, 0.3,
                      0.00,
                      0, 0.05);
    Output_Coordinate(0.20, 0.20, 0,
                      0.15, 0.15, 0,
                      0.15, 0.15, 0,
                      0.15, 0.15, 0,
                      0.30,
                      20, 0.05);
    wait(1);

    Set_Gyro_Offset(); //ジャイロのオフセット設定
    while (1)
    {
      Output_Coordinate(0.20, 0.20, 0,
                        0.15, 0.15, 0,
                        0.15, 0.15, 0,
                        0.15, 0.15, 0,
                        0.30,
                        20, 0.05);
    }
  }

  Wait_UB();

  Body_Down(0.15, 0.15);

  Wait_UB();

  Store_Position();

#ifndef ODE
  return 0;
#endif
}