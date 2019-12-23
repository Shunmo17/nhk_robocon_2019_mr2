#include "Common.h"
#include "math.h"

void Get_gyro()
{
    bno.get_angles();
    /*
    if (pc_debug == 1)
        pc.printf("BNO=(%2.2lf,%2.2lf,%2.2lf)\n", bno.euler.roll, bno.euler.pitch, bno.euler.yaw);
    pc.printf("%2.2lf, %2.2lf, %2.2lf\n", roll, pitch, yaw);
    */
    pc.printf("yaw= %2.2lf\n", yaw);

    if (bno.euler.roll == 0.0) //もしBNOが止まったら初期化
    {
        bno.setmode(OPERATION_MODE_NDOF); //BNO055初期化
        pc.printf("*********************FIX BNO************************\n");
    }

    roll = bno.euler.roll - roll_offset;
    pitch = bno.euler.pitch - pitch_offset;

    if (bno.euler.yaw > 180)
        yaw = (bno.euler.yaw - 360) - yaw_offset;
    else
        yaw = bno.euler.yaw - yaw_offset;
    if (yaw > 180)
        yaw -= 360;
    else if (yaw < -180)
        yaw += 360;

    /*
    double yaw_ = bno.euler.yaw - yaw_offset;
    if (yaw_ - yaw > 180)
        yaw = yaw_ - 360;
    else if (yaw_ - yaw < -180)
        yaw = yaw_ + 360;
    else
        yaw = yaw_;
        */
}