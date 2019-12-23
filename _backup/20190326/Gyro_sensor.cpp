#include "Common.h"
#include "math.h"

void Get_gyro()
{
    bno.get_angles();
    roll = bno.euler.roll - roll_offset;
    pitch = bno.euler.pitch - pitch_offset;
    yaw = bno.euler.yaw;
    yaw_2 = bno.euler.yaw + 360;

    /*
    if (bno.euler.yaw - yaw_old > 180)
    {
        yaw = bno.euler.yaw - 360 - yaw_offset;
    }
    else if (bno.euler.yaw - yaw_old < -180)
    {
        yaw = bno.euler.yaw + 360 - yaw_offset;
    }
    else
    {
        yaw = bno.euler.yaw - yaw_offset;
    }
    */
    //yaw=asin(bno.euler.yaw)
}