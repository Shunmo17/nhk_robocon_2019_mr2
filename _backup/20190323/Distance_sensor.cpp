#include <mbed.h>
#include "Common.h"

/**********************測距センサ 初期化***********************/
void VL53L0X_init(int number_of_vl)
{
    for (int i(0); i < number_of_vl; i++)
    {
        Xshut[i].output();
        Xshut[i] = 0;
    }
    wait(0.5);
    for (int i(0); i < number_of_vl; i++)
    {
        Xshut[i].input(); //Xshutピンに接続されたGPIOピンをインプットにするとVL53L0Xが起動
        wait(0.15);
        vl[i].init(); //初期化
        wait(0.15);
        vl[i].setAddress((uint8_t)44 + i * 2); //アドレスをデフォルトから変更しないと複数のVL53L0Xを使えない
        wait(0.15);
        vl[i].startContinuous(30);
        //VL53L0Xの使用個数に応じてループの回数を変更すること
        //存在しないVL53L0Xをいじろうとするとたぶんそこでエラーはいてとまる
    }
}

void Distance_display(int number_of_vl)
{
    for (int i(0); i < number_of_vl; i++)
    {
        vl_output[i] = vl[i].readRangeContinuousMillimeters();
    }
    oled.clear();
    for (int i(0); i < number_of_vl; i++)
    {
        oled.printf("%d mm : ", vl_output[i]);
    }
    wait(0.5);
}

