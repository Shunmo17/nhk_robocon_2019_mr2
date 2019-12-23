#include "Common.h"
#include "SO1602A.h"
#include "Distance_sensor.h"

void OLED_display()
{
    oled.clear();
    if (oled_debug == 1) //BNOと機体座標
    {
        oled.clear();
        oled.printf("BNO=(%1.0lf,%1.0lf,%1.0lf)", roll, pitch, yaw);
        oled.printf("\n(x,y)=(%1.1lf,%1.1lf)", x_now, y_now);
    }
    else if (oled_debug == 2)
    {
        oled.clear();
        //oled.printf("BNO=(%1.0lf,%1.0lf,%1.0lf)", roll, pitch, yaw);
        oled.printf("(l,r)=(%1.1lf,%1.1lf)", ratio_left, ratio_right);
        oled.printf("\ndel=%2.2lf", delta_yaw);
    }
    else if (oled_debug == 3)
    {
        oled.clear();
        oled.printf("%1d,(%1.1lf,%1.1lf)", stage, ratio_left, ratio_right);
        oled.printf("\n(x,y)=(%1.1lf,%1.1lf)", x_now, y_now);
    }
    else if (oled_debug == 4)
    {
        oled.clear();
        oled.printf("Lidar: %d cm", lidar.distance());
    }
    //pc.printf("BNO=(%2.1lf, %2.1lf, %2.1lf)\n", roll, pitch, yaw);
}