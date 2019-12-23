#include <mbed.h>
#include "Common.h"

const uint8_t PX4FLOW_ADDRESS_7bit = 0x42;
const uint8_t PX4FLOW_ADDRESS_8bit = PX4FLOW_ADDRESS_7bit << 1;

char send[1];
char recieve[22];

void Get_optical()
{
    send[0] = 0x0;//0x16

    optical_flow.write(PX4FLOW_ADDRESS_8bit, send, 1);

    wait_ms(100);

    optical_flow.read(PX4FLOW_ADDRESS_8bit, recieve, 22);//26

    int16_t flow_comp_m_x = recieve[6] + (uint16_t)(recieve[7] << 8);
    int16_t flow_comp_m_y = recieve[8] + (uint16_t)(recieve[9] << 8);
    //int16_t ground_distance = recieve[20] + (uint16_t)(recieve[21] << 8);

    //int16_t pixel_x = recieve[2] + (uint16_t)(recieve[3] << 8);
    //int16_t pixel_y = recieve[4] + (uint16_t)(recieve[5] << 8);

    //x_now_optical = pixel_y;
    //y_now_optical = pixel_x;

    x_now_optical += flow_comp_m_y * 0.001;
    y_now_optical += flow_comp_m_x * 0.001;
}