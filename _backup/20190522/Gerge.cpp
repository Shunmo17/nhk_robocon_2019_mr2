#include "Common.h"
#include "Gerge.h"

int Servo_output(double angle)
{
    return (int)(1000 * ((angle / 180) * 2.4 + 0.5));
}

void Gerge_servo_init()
{
    gerge_1.period_ms(20);
    gerge_2.period_ms(20);
}

void Wait_gerge_setting()
{
    oled.clear();
    oled.printf("gerge setting");
    wait(1);
    while (button_white == 0)
    {
        gerge_1.pulsewidth_us(Servo_output(50)); //ゲルゲ掲げる機構をオープン
        wait(0.1);
    }
    gerge_1.pulsewidth_us(Servo_output(60)); //ゲルゲ掲げる機構をロック

    wait(1);
    /*
    while (button_white == 0)
    {
        gerge_2.pulsewidth_us(Servo_output(50)); //ゲルゲ掴む機構をオープン
        wait(0.1);
    }
    gerge_2.pulsewidth_us(Servo_output(60)); //ゲルゲを掴む機構をロック
    wait(1);
    */
}

void Receive_gerge()
{
    /***************マイクロスイッチ******************/
    /*
    while (micro_switch == 0)
    {
      oled.clear();
      oled.printf("Waiting Gerge");
      wait(0.1);
    }
    */
    /*******************距離センサ********************/
    double gerge_sensor_initial(gerge_sensor.read());
    oled.clear();
    oled.printf("Waiting Gerge");
    while (abs(gerge_sensor.read() - gerge_sensor_initial) < 0.8) //最初のセンサ値との差が以上になったらゲルゲを受け取ったと認識
        wait(0.5);
    /*************************************************/
    //Hold_gerge();
    oled.clear();
    oled.printf("Get Gerge!");
    wait(1);
}

void Hold_gerge()
{
    gerge_2.pulsewidth_us(Servo_output(40)); //ゲルゲを掴む
    wait(0.5);
    gerge_2.pulsewidth_us(Servo_output(145));
}

void Raise_gerge()
{
    gerge_1.pulsewidth_us(Servo_output(30)); //ゲルゲを掲げる
}