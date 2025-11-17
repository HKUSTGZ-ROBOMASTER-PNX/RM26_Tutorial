//
// Created by ASUS on 2025/10/7.
//

#include <iterator>

#include "main.h"
#include "tx_api.h"

#include "om.h"
#include "pid.hpp"
#include "math.hpp"
#include "slope.hpp"
#include "bsp_can.hpp"
#include "magicmsgs.hpp"
#include "ServiceMotor.hpp"

using namespace Numeric;

TX_THREAD ChassisThread;
uint8_t ChassisThreadStack[4096] = {0};
uint8_t debug_mode;
int16_t dbg_cur[4];

extern CAN_HandleTypeDef hcan2;

float Vx = 0.0f;
float Vy = 0.0f;
float Vw = 0.0f;
float wheel_speed[4];

#define WHEEL_RADIUS 0.15

[[noreturn]] void ChassisThreadFun(ULONG initial_input)
{
    UNUSED(initial_input);

    om_topic_t *chassisctrl_topic = om_config_topic(nullptr, "ca", "chassisctrl", sizeof(msg_chassis_ctrl_t));
    msg_chassis_ctrl_t chassis_ctrl{};

    om_suber_t* remoter_suber = om_subscribe(om_find_topic("remoter", UINT32_MAX));
    msg_remoter_t remoter{};



    for (;;)
    {
        om_suber_export(remoter_suber, &remoter, false);

        //@TODO:save Vx, Vy, Vw with the message in the upon 'remoter' structure
        if (remoter.ctrl_sw == Relax) {
            for (uint8_t i = 0; i < 4; i++) {
                ServiceMotors::Instance()->LFWheel.currentSet = 0;
                ServiceMotors::Instance()->LRWheel.currentSet = 0;
                ServiceMotors::Instance()->RRWheel.currentSet = 0;
                ServiceMotors::Instance()->RFWheel.currentSet = 0;

            }
        }

        else if (remoter.ctrl_sw != Relax){
            Vx = remoter.left_x*0.5f;
            Vy = remoter.left_y*0.5f;
            Vw = remoter.right_x*0.5f;

            static float RelativeAngle = 0.0f;

            //@TODO: in below, use a for loop to set motors speed



            }

            ServiceMotors::Instance()->LFWheel.speedSet = wheel_speed[0];
            ServiceMotors::Instance()->LRWheel.speedSet = wheel_speed[1];
            ServiceMotors::Instance()->RRWheel.speedSet = wheel_speed[2];
            ServiceMotors::Instance()->RFWheel.speedSet = wheel_speed[3];

            ServiceMotors::Instance()->AllMotorSetOutput();

        }



        om_publish(chassisctrl_topic, &chassis_ctrl, sizeof(msg_chassis_ctrl_t), true, false);
        tx_thread_sleep(1);
    }
}
