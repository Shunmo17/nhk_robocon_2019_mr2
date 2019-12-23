#include <mbed.h>
#include "PX4Flow.h"
#include "Common.h"

int compute_velocity_x(void)
{
    // Fetch I2C data
    px4flow.update_integral();                                    // PX4FLowを初期化
    float pixel_x = px4flow.pixel_flow_x_integral() / 10.0f;      // mrad
    uint32_t timespan = px4flow.integration_timespan();           // microseconds
    int16_t ground_distance = px4flow.ground_distance_integral(); // mm

    // Scale based on ground distance and compute speed
    // (flow/1000) * (ground_distance/1000) / (timespan/1000000)
    float velocity_x = pixel_x * ground_distance / timespan; // m/s

    return velocity_x * 100; // in cm
}

int compute_velocity_y(void)
{
    // Fetch I2C data
    px4flow.update_integral();                                    // PX4FLowを初期化
    float pixel_y = px4flow.pixel_flow_y_integral() / 10.0f;      // mrad
    uint32_t timespan = px4flow.integration_timespan();           // microseconds
    int16_t ground_distance = px4flow.ground_distance_integral(); // mm

    // Scale based on ground distance and compute speed
    // (flow/1000) * (ground_distance/1000) / (timespan/1000000)
    float velocity_y = pixel_y * ground_distance / timespan; // m/s

    return velocity_y * 100; // in cm
}

void Get_px4flow(void) //距離取得関数
{
     int16_t velocity_sum_x = 0;
        int16_t velocity_sum_y = 0;

        for (uint8_t i = 1; i <= 5; i++)
        { // 5回サンプリング 100 ms待つ
            velocity_sum_x += compute_velocity_x();
            velocity_sum_y += compute_velocity_y();
            wait_ms(100);
        }

        int16_t average_velocity_x = velocity_sum_x / 5; // 平均のスピードを出す
        int16_t average_velocity_y = velocity_sum_y / 5;

    px = px + average_velocity_x;
    py = py + average_velocity_y;
}