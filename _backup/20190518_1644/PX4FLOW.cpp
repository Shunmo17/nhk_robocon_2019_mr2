#include <mbed.h>
#include "PX4Flow.h"

// 7 Bit I2C Address of the Flow Module: Default 0x42 (user selectable bits 0,1,2)
const int16_t PX4FLOW_ADDRESS = 0x42 << 1;

I2C i2c(PB_7, PB_8); // SDA 白, SCL 黄色

PX4Flow::PX4Flow()
{
}

bool PX4Flow::update()
{
    char send[1] = {0x00};
    i2c.write(PX4FLOW_ADDRESS, send, 1);
    wait_ms(10);
    char data[22];
    i2c.read(PX4FLOW_ADDRESS, data, 22);

    // read the data
    frame.frame_count = data[0] + (uint16_t)(data[1] << 8);
    frame.pixel_flow_x_sum = data[2] + (uint16_t)(data[3] << 8);
    frame.pixel_flow_y_sum = data[4] + (uint16_t)(data[5] << 8);
    frame.flow_comp_m_x = data[6] + (uint16_t)(data[7] << 8);
    frame.flow_comp_m_y = data[8] + (uint16_t)(data[9] << 8);
    frame.qual = data[10] + (uint16_t)(data[11] << 8);
    frame.gyro_x_rate = data[12] + (uint16_t)(data[13] << 8);
    frame.gyro_y_rate = data[14] + (uint16_t)(data[15] << 8);
    frame.gyro_z_rate = data[16] + (uint16_t)(data[17] << 8);
    frame.gyro_range = data[18];
    frame.sonar_timestamp = data[19];
    frame.ground_distance = data[20] + (uint16_t)(data[21] << 8);

    return true;
}

bool PX4Flow::update_integral()
{
    char send[1] = {0x16};
    i2c.write(PX4FLOW_ADDRESS, send, 1);
    wait_ms(10);
    char data[26];
    i2c.read(PX4FLOW_ADDRESS, data, 26);

    // read the data
    iframe.frame_count_since_last_readout = data[0] + (uint16_t)(data[1] << 8);
    iframe.pixel_flow_x_integral = data[2] + (uint16_t)(data[3] << 8);
    iframe.pixel_flow_y_integral = data[4] + (uint16_t)(data[5] << 8);
    iframe.gyro_x_rate_integral = data[6] + (uint16_t)(data[7] << 8);
    iframe.gyro_y_rate_integral = data[8] + (uint16_t)(data[9] << 8);
    iframe.gyro_z_rate_integral = data[10] + (uint16_t)(data[11] << 8);
    iframe.integration_timespan = data[12] + (uint16_t)(data[13] << 8) + (uint32_t)(data[14] << 16) + (uint32_t)(data[15] << 24);
    iframe.sonar_timestamp = data[16] + (uint16_t)(data[17] << 8) + (uint32_t)(data[18] << 16) + (uint32_t)(data[19] << 24);
    iframe.ground_distance = data[20] + (uint16_t)(data[21] << 8);
    iframe.gyro_temperature = data[22] + (uint16_t)(data[23] << 8);
    iframe.quality = data[24];

    return true;
}

// Simple frame
uint16_t PX4Flow::frame_count()
{
    return frame.frame_count;
}

int16_t PX4Flow::pixel_flow_x_sum()
{
    return frame.pixel_flow_x_sum;
}

int16_t PX4Flow::pixel_flow_y_sum()
{
    return frame.pixel_flow_y_sum;
}

int16_t PX4Flow::flow_comp_m_x()
{
    return frame.flow_comp_m_x;
}

int16_t PX4Flow::flow_comp_m_y()
{
    return frame.flow_comp_m_y;
}

int16_t PX4Flow::gyro_x_rate()
{
    return frame.gyro_x_rate;
}

int16_t PX4Flow::gyro_y_rate()
{
    return frame.gyro_y_rate;
}

int16_t PX4Flow::gyro_z_rate()
{
    return frame.gyro_z_rate;
}

int16_t PX4Flow::qual()
{
    return frame.qual;
}

uint8_t PX4Flow::sonar_timestamp()
{
    return frame.sonar_timestamp;
}

int16_t PX4Flow::ground_distance()
{
    return frame.ground_distance;
}

// Integral frame
uint16_t PX4Flow::frame_count_since_last_readout()
{
    return iframe.frame_count_since_last_readout;
}

int16_t PX4Flow::pixel_flow_x_integral()
{
    return iframe.pixel_flow_x_integral;
}

int16_t PX4Flow::pixel_flow_y_integral()
{
    return iframe.pixel_flow_y_integral;
}

int16_t PX4Flow::gyro_x_rate_integral()
{
    return iframe.gyro_x_rate_integral;
}

int16_t PX4Flow::gyro_y_rate_integral()
{
    return iframe.gyro_y_rate_integral;
}

int16_t PX4Flow::gyro_z_rate_integral()
{
    return iframe.gyro_z_rate_integral;
}

uint32_t PX4Flow::integration_timespan()
{
    return iframe.integration_timespan;
}

uint32_t PX4Flow::sonar_timestamp_integral()
{
    return iframe.sonar_timestamp;
}

int16_t PX4Flow::ground_distance_integral()
{
    return iframe.ground_distance;
}

int16_t PX4Flow::gyro_temperature()
{
    return iframe.gyro_temperature;
}

uint8_t PX4Flow::quality_integral()
{
    return iframe.quality;
}