#include "Common.h"
#include "SO1602A.h"

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
        //oled.printf("BNO=(%1.0lf,%1.0lf,%1.0lf)", roll, pitch, yaw);
        oled.printf("(r,l)=(%1.2lf,%1.2lf)", ratio_right, ratio_left);
        oled.printf("\nref=%2.0lf,now=%2.0lf", yaw_ref, yaw);
    }
}