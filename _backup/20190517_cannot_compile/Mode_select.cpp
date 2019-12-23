#include <mbed.h>
#include "Common.h"

#define max_mode 5
#define max_state 4

void Mode_display();

void State_display(void)
{
    oled.clear();
    if (state == 1)
        oled.printf("[State]1\nStart Urutu1");
    else if (state == 2)
        oled.printf("[State]2\nBefore Sand");
    else if (state == 3)
        oled.printf("[State]3\nBefore Rope");
    else if (state == 4)
        oled.printf("[State]4\nStart Urutu2");
    else
        oled.printf("Error");
}

/*******************ボタンに対応して変数を変える関数***************/
void Change_mode(void)
{
    if (mode < max_mode)
        mode++;
    else
        mode = 0;
    Mode_display();
    wait(0.05);
}

void Invert_yaw_mode(void)
{
    yaw_mode = !yaw_mode;
    oled.init();
    Mode_display();
    wait(0.05);
}

void Change_state(void)
{
    if (state < max_state)
        state++;
    else
        state = 1;
    State_display();
    wait(0.05);
}

/*******************************************************/

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

    oled.clear();
    if (mode == 0)
        oled.printf("[Mode]Walking");
    else if (mode == 1)
        oled.printf("[Mode]MR2 All");
    else if (mode == 2)
        oled.printf("[Mode]Slope");
    else if (mode == 3)
        oled.printf("[Mode]Stand");
    else if (mode == 4)
        oled.printf("[Mode]Timer");
    else if (mode == 5)
        oled.printf("[Mode]TEST");
    else
        oled.printf("Error");

    if (yaw_mode == 1)
        oled.printf("\n[Yaw]ON");
    else
        oled.printf("\n[Yaw]OFF");
}

void Mode_select()
{
    Mode_display();
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
        if (button_red.read() == 1)
        {
            Change_state();
        }
        if (button_white.read() == 1)
            break;
        wait(0.1);
    }
}