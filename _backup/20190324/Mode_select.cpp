#include <mbed.h>
#include "Common.h"

#define max_mode 6
#define max_oled_debug 2

void Mode_display();
void Change_mode();
void Invert_yaw_mode();
void Change_oled_debug();

void Mode_select()
{
    Mode_display();
    while (1)
    {
        if (button_blue_right.read() == 1)
        {
            Change_mode();
            pc.printf("pushed blue_right\n");
        }
        if (button_blue_left.read() == 1)
        {
            Invert_yaw_mode();
            pc.printf("pushed blue_left\n");
        }
        if (button_white.read() == 1)
        {
            break;
        }
        wait(0.1);
    }
}

void Mode_display(void)
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
    else if (mode == 1)
    {
        oled.printf("[Mode]Sand");
    }
    else if (mode == 2)
    {
        oled.printf("[Mode]Slope");
    }
    else if (mode == 6)
    {
        oled.printf("[Mode]Stand");
    }
    else
    {
        oled.printf("Error");
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
    Mode_display();
    wait(0.05);
}

void Invert_yaw_mode(void)
{
    yaw = !yaw;
    oled.init();
    Mode_display();
    wait(0.05);
}

void Change_oled_debug(void)
{
    oled_debug++;
    oled.init();

    wait(0.05);
}