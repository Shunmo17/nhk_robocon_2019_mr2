#include "Common.h"

int Servo::angle(double angle_ref) //サーボの目標角度からPWM信号を出す関数
{
#ifdef SERVO_LIMIT
    if (angle_ref < angle_min)
    {
        //pc.printf("under angle\n");
        angle_ref = angle_min;
    }
    else if (angle_ref > angle_max)
    {
        //pc.printf("over angle = %lf\n", angle_ref - angle_max);
        angle_ref = angle_max;
    }
#endif
    double pos(0.0);
    double pulse(0.0);
    pos = 7.8853 * (angle_ref + angle_offset) + 603.24;
    pulse = (pos / 20000) * 4096 - 1;
    //pc.printf("pos= %lf\n", pos);
    //pc.printf("pulse= %lf\n", pulse);
    return (int)pulse;
}

Servo::Servo(void)
{
    angle_min = angle_max = angle_offset = 0.0;
    angle_old = angle_ref = angle_now = 0.0;
}