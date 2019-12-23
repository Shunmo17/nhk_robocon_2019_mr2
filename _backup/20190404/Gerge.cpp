#include "Common.h"
#include "Gerge.h"

double Servo_output(double angle)
{
    return (angle / 180) * 2.4 + 0.5;
}

void Gerge_servo_init()
{
    gerge_1.period_ms(20);
    gerge_1.period_ms(20);
}

void Wait_gerge_setting()
{
    while (button_white == 0)
    {
        gerge_1.pulsewidth_ms(Servo_output(10)); //この間にゲルゲ掴む機構をセッティング
        wait(0.1);
    }
    gerge_1.pulsewidth_ms(Servo_output(40)); //ゲルゲ掴む機構をロック
    wait(1);
    while (button_white == 0)
    {
        gerge_2.pulsewidth_ms(Servo_output(10)); //この間にゲルゲ掲げる機構をセッティング
        wait(0.1);
    }
    gerge_2.pulsewidth_ms(Servo_output(40)); //ゲルゲ掲げる機構をロック
}

void Hold_gerge()
{
    gerge_1.pulsewidth_ms(Servo_output(10)); //ゲルゲを掴む
}

void Raise_gerge()
{
    gerge_2.pulsewidth_ms(Servo_output(40)); //ゲルゲを掲げる
}