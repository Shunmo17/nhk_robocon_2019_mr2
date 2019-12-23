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
        oled.printf("\n(x,y)=(%1.1lf,%1.1lf)", bodyposition[0], bodyposition[1]);
    }
    else if (oled_debug == 2)
    {
        oled.clear();
        oled.printf("BNO=(%1.0lf,%1.0lf,%1.0lf)", roll, pitch, yaw);
        oled.printf("\n(l,r)=(%1.1lf,%1.1lf)", ratio_left, ratio_right);
    }
    else if (oled_debug == 3)
    {
        oled.clear();
        oled.printf("%1d,(%1.1lf,%1.1lf)", stage, ratio_left, ratio_right);
        oled.printf("\n(x,y)=(%1.1lf,%1.1lf)", bodyposition[0], bodyposition[1]);
    }
    else if (oled_debug == 4)
    {
        Get_distance();
        oled.clear();
        oled.printf("dis:(%d,%d)", vl_output[0], vl_output[1]);
    }
}