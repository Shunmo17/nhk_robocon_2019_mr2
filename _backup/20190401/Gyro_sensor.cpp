#include "Common.h"
#include "math.h"

void Get_gyro()
{
    bno.get_angles();
    roll = bno.euler.roll - roll_offset;
    pitch = bno.euler.pitch - pitch_offset;
    double yaw_ = bno.euler.yaw - yaw_offset;
    if (yaw_ - yaw > 180)
        yaw = yaw_ - 360;
    else if (yaw_ - yaw < -180)
        yaw = yaw_ + 360;
    else
        yaw = yaw_;
    //yaw_2 = bno.euler.yaw + 360;
    //yaw_3 = bno.euler.yaw - 360;
    //pc.printf("%lf,%lf,%lf\n", roll, pitch, yaw);

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