//
// Created by cosmosmount on 2025/9/10.
//

#include "bsp_dwt.hpp"
#include "main.h"
#include "tx_api.h"

#include "om.h"
#include "pid.hpp"
#include "crc.hpp"
#include "math.hpp"
#include "filter.hpp"
#include "magicmsgs.hpp"

using namespace Filter;
using namespace Numeric;

TX_THREAD GimbalThread;
uint8_t GimbalThreadStack[4096] = {0};

extern TX_SEMAPHORE IMUThreadSem;

#define GimbalTest
#ifdef GimbalTest


double yaw_signal_step = 0.82f;
double pitch_signal_T = 0.2f;

static double signal(double t)
{
    constexpr double T = 0.6;           // 周期

    constexpr double buffer_ratio = 1.0 / 5.0;
    constexpr double rise_ratio = 4.0 / 5.0;

    const double t_mod = std::fmod(t, T);

    constexpr double buffer_time = buffer_ratio * T;
    constexpr double rise_time = rise_ratio * T;

    if (t_mod < buffer_time)
    {
        // 前1/5缓冲段
        return 0.0;
    }
    else if (t_mod < T)
    {
        double step_value = yaw_signal_step;

        const double rise_t = t_mod - buffer_time;
        const double progress = rise_t / rise_time; // 0 ~ 1

        const double smooth = (1 - std::cos(Numeric::Pi * progress)) / 2.0;

        return step_value * smooth;
    }
    else
    {
        return 0.0;
    }
}

static double tri_signal(double t)
{
    constexpr double T = 0.6;           // 周期

    constexpr double buffer_ratio = 1.0 / 5.0;
    constexpr double rise_ratio = 4.0 / 5.0;

    const double t_mod = std::fmod(t, T);

    constexpr double buffer_time = buffer_ratio * T;
    constexpr double rise_time = rise_ratio * T;

    if (t_mod < buffer_time)
    {
        // 前1/5缓冲段
        return 0.0;
    }
    else if (t_mod < T)
    {
        double step_value = yaw_signal_step;
        // 后4/5 线性上升段（三角波）
        const double rise_t = t_mod - buffer_time;
        const double progress = rise_t / rise_time; // 0 ~ 1

        // 线性
        const double smooth = progress;

        return step_value * smooth;
    }
    else
    {
        return 0.0;
    }
}

static double sin_signal(double t)
{
    const double amplitude = 0.03;  // 振幅（小幅度）
    double T = pitch_signal_T;          // 周期（快速振动）
    const double omega = 2.0 * M_PI / T; // 角频率 ω = 2π / T

    return amplitude * std::sin(omega * t);
}


[[noreturn]] void GimbalThreadFun(ULONG initial_input)
{


    for (;;)
    {
        //only if IMU have send data, start gimbal control
        if (tx_semaphore_get(&IMUThreadSem, TX_WAIT_FOREVER) == TX_SUCCESS)
        {

        }
    }
}