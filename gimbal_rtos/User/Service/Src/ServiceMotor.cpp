#include "main.h"

#include "ServiceMotor.hpp"
#include "om.h"
#include "magicmsgs.hpp"
#include "filter.hpp"
#include "math.hpp"
#include "bsp_dwt.hpp"

using namespace Filter;
using namespace Numeric;

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
extern CAN_HandleTypeDef hcan3;

TX_SEMAPHORE MotorCANRecvSem;

TX_THREAD MotorThread;
uint8_t MotorThreadStack[4096] = {0};
DJIMotorHandler* DJIMotorhandler = DJIMotorHandler::Instance();

ctrl_debug_t motor_debug;
pid_tuning_t motor_pos_pid;
pid_tuning_t motor_spd_pid;
float debug_cur = 0.0f;
float vel_ratio = 0;
float test_r = 0;
float debug_set = 0.0f;

ServiceMotors *serviceMotors = ServiceMotors::Instance();


void ServiceMotors::MotorRegister() {
    // //注册电机
    DJIMotorhandler->registerMotor(&LFWheel, &hcan1, 0x201);
    LFWheel.controlMode = DJIMotor::SPD_MODE;
    LFWheel.gearBox = GearBox_M3508;
    LFWheel.currentSet = 0;
    DJIMotorhandler->registerMotor(&LRWheel, &hcan1, 0x202);
    LRWheel.controlMode = DJIMotor::SPD_MODE;
    LRWheel.gearBox = GearBox_M3508;
    LRWheel.currentSet = 0;
    DJIMotorhandler->registerMotor(&RRWheel, &hcan1, 0x203);
    RRWheel.controlMode = DJIMotor::SPD_MODE;
    RRWheel.gearBox = GearBox_M3508;
    RRWheel.currentSet = 0;
    DJIMotorhandler->registerMotor(&RFWheel, &hcan1, 0x204);
    RFWheel.controlMode = DJIMotor::SPD_MODE;
    RFWheel.gearBox = GearBox_M3508;
    RFWheel.currentSet = 0;

}

void ServiceMotors::SetModeAndPidParam()
{
    LFWheel.speedPid.kp = 800.0f;
    LFWheel.speedPid.ki = 0.0f;
    LFWheel.speedPid.kd = 1.0f;

    LRWheel.speedPid.kp = 800.0f;
    LRWheel.speedPid.ki = 0.0f;
    LRWheel.speedPid.kd = 1.0f;

    RRWheel.speedPid.kp = 800.0f;
    RRWheel.speedPid.ki = 0.0f;
    RRWheel.speedPid.kd = 1.0f;

    RFWheel.speedPid.kp = 800.0f;
    RFWheel.speedPid.ki = 0.0f;
    RFWheel.speedPid.kd = 1.0f;
}

void ServiceMotors::AllMotorSetOutput() {
    LFWheel.setOutput();
    LRWheel.setOutput();
    RRWheel.setOutput();
    RFWheel.setOutput();
}

[[noreturn]] void MotorThreadFun(ULONG initial_input) {
    UNUSED(initial_input);
    ULONG time;

    //注册电机
    ServiceMotors::Instance()->MotorRegister();

    om_suber_t *chassis_suber = om_subscribe(om_find_topic("chassisctrl", UINT32_MAX));
    msg_chassis_ctrl_t chassis_ctrl{};

    ServiceMotors::Instance()->SetModeAndPidParam();

    motor_pos_pid.kp = 10.0f;
    motor_pos_pid.ki = 0.0f;
    motor_pos_pid.kd = 0.0f;

    motor_spd_pid.kp = 300.0f;
    motor_spd_pid.ki = 0.01f;
    motor_spd_pid.kd = 1.0f;
    float yaw_init = 0.0f;

    for (;;) {
        om_suber_export(chassis_suber, &chassis_ctrl, false);

        // for (int i = 0; i < 4; i++) {
        //     serviceMotors->WheelMotors[i]->currentSet = chassis_ctrl.wheel_cur[i];
        // }
        //发送控制指令给电机
        DJIMotorhandler->sendControlData();

        tx_thread_sleep(1);
    }
}

/**
 * @brief CAN接收中断回调函数，所有反馈在can上的数据会在这里根据ID进行分类并处理。
 * @param hcan CAN句柄
 * @note 该函数用于处理CAN接收中断，根据ID分类处理接收到的数据。但是这中方法可能会在回调里浪费时间，因为这里的处理是阻塞的。考虑是否需要将数据存储到一个缓冲区，然后在主循环中处理。
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    // 接收数据
    uint8_t rx_data[8];
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);

    /*-------------------------------------------------大疆电机数据-------------------------------------------------*/
    if (rx_header.StdId >= 0x201 && rx_header.StdId <= 0x208)
    {
        if (hcan->Instance == CAN1)
        {
            DJIMotorHandler::Instance()->updateFeedback(hcan, rx_data, int(rx_header.StdId - 0x201));
        }
        else if (hcan->Instance == CAN2) // 处理CAN2的数据
        {
            DJIMotorHandler::Instance()->updateFeedback(hcan, rx_data, int(rx_header.StdId - 0x201));
        }
    }
    else // 未知的ID，需要进行错误处理
    {
    }
}