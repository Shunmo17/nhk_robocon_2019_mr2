#include <mbed.h>
#include "Common.h"

#define max_mode 2
#define max_oled_debug 2

void OLED_display();
void Change_mode();
void Invert_yaw_mode();
void Change_oled_debug();

void Mode_select()
{
    OLED_display();
    while (1)
    {
        if (button_blue_right.read() == 1)
        {
            Change_mode();
        }
        if (button_blue_left.read() == 1)
        {
            Invert_yaw_mode();
        }
        if (button_white.read() == 1)
        {
            break;
        }
        wait(1);
    }
}

void OLED_display(void)
{
    /***************操作音*****************/
    /*
  speaker.period(1.0 / mC);
  speaker.write(0.5f);
  wait(0.1f);
  speaker.write(0.0f);
  */
    /**************************************/

    oled.init();
    if (mode == 0)
    {
        oled.printf("[Mode]Walking");
    }
    if (mode == 1)
    {
        oled.printf("[Mode]Sand");
    }
    if (mode == 2)
    {
        oled.printf("[Mode]Slope");
    }
    if(mode==6){
        oled.printf("[Mode]Stand");
    }
    if (yaw == 1)
    {
        oled.printf("\n[Yaw]ON");
    }
    else
    {
        oled.printf("\n[Yaw]OFF");
    }
}

/*******************ボタンに対応して変数を変える関数***************/
void Change_mode(void)
{
    if (mode < max_mode)
    {
        mode++;
    }
    else
    {
        mode = 0;
    }
    OLED_display();
    wait(0.05);
}

void Invert_yaw_mode(void)
{
    yaw = !yaw;
    oled.init();
    OLED_display();
    wait(0.05);
}

void Change_oled_debug(void)
{
    oled_debug++;
    oled.init();

    wait(0.05);
}