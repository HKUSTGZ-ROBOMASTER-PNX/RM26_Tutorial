//
// Created by ASUS on 2025/10/25.
//
#include "crc.hpp"
#include "maix.hpp"

#include <string.h>

void Maix::Init()
{
    memset(maix_rx.buffer, 0, sizeof(maix_rx));
}

void Maix::Update()
{
    if (Verify_CRC16_Check_Sum(maix_rx.buffer, sizeof(maix_rx.buffer)))
    {
        memcpy(prev_rx, maix_rx.buffer, sizeof(maix_rx.buffer));
    }
    else
    {
        memcpy(prev_rx, maix_rx.buffer, sizeof(maix_rx.buffer));
    }
}

