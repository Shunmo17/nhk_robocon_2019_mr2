#include "Common.h"
#include "SO1602A.h"
#include "Distance_sensor.h"

void OLED_display()
{
    oled.clear();
    if (oled_debug == 1)
    {
        oled.clear();
        oled.printf("(%1.0lf,%1.0lf,%1.0lf)", roll, pitch, yaw);
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
        oled.printf("(%1.1lf,%1.1lf)", ratio_left, ratio_right);
        oled.printf("\n(x,y)=(%1.1lf,%1.1lf)", x_now, y_now);
    }
    else if (oled_debug == 4)
    {
        oled.clear();
        oled.printf("(%1.1lf,%1.1lf,%1.1lf)", roll, pitch, yaw);
        //oled.printf("\nLidar: %d cm", lidar.distance());
    }
    else if (oled_debug == 5)
    {
        oled.clear();
        oled.printf("opt=(%1.1lf,%1.1lf)", x_now_optical, y_now_optical);
        //oled.printf("\nleg=(%1.1lf,%1.1lf)", x_now, y_now);
    }
    //pc.printf("BNO=(%2.1lf, %2.1lf, %2.1lf)\n", roll, pitch, yaw);
}