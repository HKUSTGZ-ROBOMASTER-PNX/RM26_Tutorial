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
msg_remoter_t debug_dr16;
msg_ins_t debug_ins;
msg_visionrx_t debug_vrx;
ctrl_debug_t yaw_debug;
ctrl_debug_t pitch_debug;

float debug_dyaw;

// 200 0 0; 50 0 1500
PID yaw_pos_pid(200.0f, 0.0f, 0.0f, 500.0f, 10.0f, PID_POSITION | PID_Derivative_On_Measurement);
PID yaw_spd_pid(40.0f, 0.0f, 1800.0f, 500.0f, 100.0f, PID_POSITION);

PID pitch_pos_pid(200.0f, 0.0f, 0.0f, 500.0f, 0.0f, PID_POSITION);
PID pitch_spd_pid(200.0f, 0.0f, 1000.0f, 500.0f, 0.0f, PID_POSITION);
#endif

// #define NonvisionTest
#ifdef NonvisionTest
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

#endif

[[noreturn]] void GimbalThreadFun(ULONG initial_input)
{
    UNUSED(initial_input);

    float dyaw = 0.0f;
    bool autoaim = false;
    bool valid_vision_rx = false;

    om_topic_t *gimbalctrl_topic = om_config_topic(nullptr, "ca", "gimbalctrl", sizeof(msg_gimbal_ctrl_t));
    msg_gimbal_ctrl_t gimbal_ctrl{};
    om_suber_t *remoter_suber = om_subscribe(om_find_topic("remoter", UINT32_MAX));
    msg_remoter_t remoter{};
    om_suber_t *ins_suber = om_subscribe(om_find_topic("ins", UINT32_MAX));
    msg_ins_t ins{};
    om_suber_t *vision_suber = om_subscribe(om_find_topic("visionrx", UINT32_MAX));
    msg_visionrx_t vision_rx{};
    msg_visionrx_t prev_vision_rx{};
    memset(&prev_vision_rx, 0, sizeof(msg_visionrx_t));

    KalmanFilter vision_yaw_filter;
    KalmanFilter vision_pitch_filter;

    vision_yaw_filter.SetQ(0.00001);
    vision_pitch_filter.SetQ(0.00001);
    vision_yaw_filter.SetR(0.0001);
    vision_pitch_filter.SetR(0.0001);

    for (;;)
    {
        dyaw = 0.0f;
        valid_vision_rx = false;

        om_suber_export(remoter_suber, &remoter, false);
        om_suber_export(ins_suber, &ins, false);
        om_suber_export(vision_suber, &vision_rx, false);
        memcpy (&debug_dr16, &remoter, sizeof(msg_remoter_t));
        memcpy (&debug_ins, &ins, sizeof(msg_ins_t));

        if (Verify_CRC16_Check_Sum(reinterpret_cast<uint8_t*>(&vision_rx), sizeof(msg_visionrx_t)))
        {
            if (isnan(vision_rx.yaw) || isnan(vision_rx.yaw_vel) || isnan(vision_rx.yaw_acc) ||
                isnan(vision_rx.pitch) || isnan(vision_rx.pitch_vel) || isnan(vision_rx.pitch_acc) ||
                vision_rx.project_x < 0.0f || vision_rx.project_y > 1.0f)
            {
                valid_vision_rx = false;
                memcpy(&vision_rx, &prev_vision_rx, sizeof(msg_visionrx_t));
            }
            else
            {
                valid_vision_rx = true;
                memcpy(&prev_vision_rx, &vision_rx, sizeof(msg_visionrx_t));
            }
        }

        memcpy (&debug_vrx, &vision_rx, sizeof(msg_visionrx_t));

        if (tx_semaphore_get(&IMUThreadSem, TX_WAIT_FOREVER) == TX_SUCCESS)
        {
            if (remoter.ctrl_sw == Relax || remoter.offline)
            {
                gimbal_ctrl.yaw_mode = TORQUE;
                gimbal_ctrl.pitch_mode = TORQUE;

                gimbal_ctrl.yaw_torque = 0.0f;
                gimbal_ctrl.pitch_torque = 0.0f;

#ifdef GimbalTest
                dyaw = vision_yaw_filter.Update(vision_rx.yaw) - ins.yaw*DegreeToRad;
                if (dyaw > Pi)
                    dyaw -= PiX2;
                else if (dyaw < -Pi)
                    dyaw += PiX2;
                if (dyaw > Pi * 0.5 || dyaw < -Pi * 0.5)
                    dyaw = 0.0f;
#endif
            }
            else
            {
                if (valid_vision_rx)
                {
                    gimbal_ctrl.yaw_mode = TORQUE;
                    gimbal_ctrl.pitch_mode = TORQUE;

                    // yaw_pos_pid.ref = ins.total_yaw*DegreeToRad
                    // + LoopFloatConstrain(vision_yaw_filter.Update(vision_rx.yaw) - ins.yaw*DegreeToRad, -Pi, Pi);

                    dyaw = vision_yaw_filter.Update(vision_rx.yaw) - ins.yaw*DegreeToRad;
                    if (dyaw > Pi)
                        dyaw -= PiX2;
                    else if (dyaw < -Pi)
                        dyaw += PiX2;
                    if (dyaw > Pi * 0.5 || dyaw < -Pi * 0.5)
                        dyaw = 0.0f;

                    yaw_pos_pid.ref = ins.total_yaw*DegreeToRad + dyaw;
                    pitch_pos_pid.ref = vision_pitch_filter.Update(vision_rx.pitch);
                }
                else
                {
                    // gimbal_ctrl.yaw_speed = remoter.right_x*7.0f;
                    // gimbal_ctrl.pitch_speed = remoter.right_y*7.0f;
                    // gimbal_ctrl.yaw_mode = SPD;
                    // gimbal_ctrl.pitch_mode = SPD;
                    yaw_pos_pid.ref = ins.total_yaw*DegreeToRad;
                    pitch_pos_pid.ref = ins.pitch*DegreeToRad;
                }

                yaw_pos_pid.fdb = ins.total_yaw*DegreeToRad;
                yaw_pos_pid.UpdateResult();
                yaw_spd_pid.ref = yaw_pos_pid.result;
                yaw_spd_pid.fdb = ins.gyro_y;
                yaw_spd_pid.UpdateResult();
                gimbal_ctrl.yaw_torque = yaw_spd_pid.result;

                pitch_pos_pid.fdb = ins.pitch*DegreeToRad;
                pitch_pos_pid.UpdateResult();
                pitch_spd_pid.ref = pitch_pos_pid.result;
                pitch_spd_pid.fdb = ins.gyro_p;
                pitch_pos_pid.UpdateResult();
                gimbal_ctrl.pitch_torque = pitch_spd_pid.result;
            }

#ifdef GimbalTest
            // yaw_pos_pid.Tuning(yaw_pos_tuning.kp, yaw_pos_tuning.ki, yaw_pos_tuning.kd);
            // yaw_spd_pid.Tuning(yaw_spd_tuning.kp, yaw_spd_tuning.ki, yaw_spd_tuning.kd);
            // pitch_pos_pid.Tuning(pitch_pos_tuning.kp, pitch_pos_tuning.ki, pitch_pos_tuning.kd);
            // pitch_spd_pid.Tuning(pitch_spd_tuning.kp, pitch_spd_tuning.ki, pitch_spd_tuning.kd);
            yaw_debug.vis_rec = vision_rx.yaw;
            yaw_debug.vis_set = ins.total_yaw*DegreeToRad + dyaw;
            debug_dyaw = dyaw;
            yaw_debug.pos_set = yaw_pos_pid.ref;
            yaw_debug.pos_fdb = ins.total_yaw*DegreeToRad;
            yaw_debug.spd_set = yaw_spd_pid.ref;
            yaw_debug.spd_fdb = yaw_spd_pid.fdb;

            pitch_debug.vis_rec = vision_rx.pitch;
            pitch_debug.pos_set = pitch_pos_pid.ref;
            pitch_debug.pos_fdb = pitch_pos_pid.fdb;
            pitch_debug.spd_set = pitch_spd_pid.ref;
            pitch_debug.spd_fdb = pitch_spd_pid.fdb;
#endif

            om_publish(gimbalctrl_topic, &gimbal_ctrl, sizeof(msg_gimbal_ctrl_t), true, false);
            tx_thread_sleep(1);
        }
    }
}