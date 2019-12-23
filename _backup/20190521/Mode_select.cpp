#include <mbed.h>
#include "Common.h"

#define max_mode 6
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
    oled.clear();
    if (mode == ALL_1)
        oled.printf("[Mode]ALL_1");
    else if (mode == ALL_2)
        oled.printf("[Mode]ALL_2");
    else if (mode == WALKING)
        oled.printf("[Mode]Walking");
    else if (mode == STAND)
        oled.printf("[Mode]Stand");
    else if (mode == SENSOR)
        oled.printf("[Mode]Sensor");
    else if (mode == PX4FLOW)
        oled.printf("[Mode]PX4FLOW");
    else if (mode == OFFSET)
        oled.printf("[Mode]Offset");
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
    wait(1);
    while (1)
    {
        if (button_blue_right.read() == 1)
            Change_mode();
        if (button_blue_left.read() == 1)
            Invert_yaw_mode();
        if (button_red.read() == 1)
            Change_state();
        if (button_white.read() == 1)
            break;
        wait(0.05);
    }
}