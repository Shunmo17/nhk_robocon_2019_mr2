#include "Common.h"
#include "math.h"

void Get_gyro()
{
    bno.get_angles();
    double yaw_fix(0.0);
    if (pc_debug == 3)
        pc.printf("%2.2lf\n", roll);

    if (bno.euler.roll == 0.0 && bno.euler.pitch == 0.00 && bno.euler.yaw == 0.0) //もしBNOが止まったら初期化
    {
        yaw_fix = yaw;
        bno.setmode(OPERATION_MODE_IMUPLUS); //BNO055初期化
        pc.printf("*********************FIX BNO************************\n");
    }

    roll = bno.euler.roll - roll_offset;
    pitch = bno.euler.pitch - pitch_offset;

    if (bno.euler.yaw > 180)
        yaw = (bno.euler.yaw - 360) - yaw_offset + yaw_fix;
    else
        yaw = bno.euler.yaw - yaw_offset + yaw_fix;
    if (yaw > 180)
        yaw -= 360;
    else if (yaw < -180)
        yaw += 360;
}