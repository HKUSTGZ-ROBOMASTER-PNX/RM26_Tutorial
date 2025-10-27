//
// Created by ASUS on 2025/10/25.
//

#ifndef FRAMEWORK_MAIX_HPP
#define FRAMEWORK_MAIX_HPP

#include "main.h"

#define MAIX_BUFFER_SIZE 12

struct maix_comm_data_t
{
    uint8_t header;   // 数据包头
    uint8_t Vx;       // X轴速度
    uint8_t Vy;       // Y轴速度
    uint8_t Vw;       // 角速度
    uint8_t M1, M2, M3, M4, M5, M6;  // 6个舵机控制值
    uint16_t crc;     // CRC校验值
} __attribute__((packed));

union maix_comm_rx_t
{
    maix_comm_data_t data;
    uint8_t buffer[sizeof(maix_comm_data_t)];
};

class Maix
{
public:
    void Init();
    void Update();
    maix_comm_rx_t maix_rx;
    static Maix *Instance()
    {
        static Maix instance;
        return &instance;
    }
private:
    uint8_t prev_rx[MAIX_BUFFER_SIZE] = {0};
};

#endif //FRAMEWORK_MAIX_HPP