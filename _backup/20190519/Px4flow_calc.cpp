#include <mbed.h>
#include "PX4Flow.h"
#include "Common.h"

Timer t;                     // タイマtをセット

void compute_velocity(float &velocity_x, float &velocity_y)
{ // 速度を出す関数 参照で値を返す
    static uint32_t last_check = 0;
    uint32_t loop_start = t.read_ms();

    if (loop_start - last_check > 100)
    {
        // Fetch I2C data
        px4flow.update_integral();                                    // PX4FLow
        float pixel_x = px4flow.pixel_flow_x_integral() / 10.0f;      // mrad
        float pixel_y = px4flow.pixel_flow_y_integral() / 10.0f;      // mrad
        uint32_t timespan = px4flow.integration_timespan();           // microseconds
        int16_t ground_distance = px4flow.ground_distance_integral(); // mm

        // Scale based on ground distance and compute speed
        // (flow/1000) * (ground_distance/1000) / (timespan/1000000)
        velocity_x = pixel_x * ground_distance * 100 / timespan; // cm/s
        velocity_y = pixel_y * ground_distance * 100 / timespan; // cm/s
    }
    else
    {
        velocity_x = 0;
        velocity_y = 0;
    }

    last_check = loop_start;
}

void Get_px4flow(void) //距離取得関数
{
    const uint8_t sampling_number = 10; // サンプリング数
    t.start();                          // タイマtをスタート

    while (true)
    {
        int16_t velocity_sum_x = 0;
        int16_t velocity_sum_y = 0;

        for (uint8_t i = 0; i < sampling_number; i++)
        { // サンプ
            float velocity_x = 0;
            float velocity_y = 0;

            compute_velocity(velocity_x, velocity_y);
            velocity_sum_x += (int16_t)velocity_x;
            velocity_sum_y += (int16_t)velocity_y;

            wait_ms(30); // PX4Flowは最大40Hz = 25 msなので余裕を持って30 ms待つ
        }

        int16_t average_velocity_x = velocity_sum_x / sampling_number; // 平均のスピードを出す
        int16_t average_velocity_y = velocity_sum_y / sampling_number;

        px = px + average_velocity_x;
        py = py + average_velocity_y;
    }
}