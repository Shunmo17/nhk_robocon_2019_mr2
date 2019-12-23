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
    while (gerge_sensor.read() > 0.55)
    {
        oled.clear();
        oled.printf("Waiting Gerge");
        pc.printf("gerge_distance=%2.2lf\n", gerge_sensor.read());
        wait(0.1);
    }
    /*************************************************/
    //Hold_gerge();
    oled.clear();
    oled.printf("Get Gerge!");
    wait(2);
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