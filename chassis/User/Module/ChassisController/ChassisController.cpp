#include "ChassisController.hpp"
#include "bsp_usart.hpp"
#include "bsp_dwt.hpp"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

/**
 * @brief 初始化函数，用于挂载四个底盘电机，并设置PID参数
 */
void ChassisController::Init()
{
    /*-----------------------初始化一些基本数值-----------------------*/
    Vx = 0;
    Vy = 0;
    Vw = 0;
    RelativeAngle = 0;

    RF_speedSet = 0;
    LF_speedSet = 0;
    RR_speedSet = 0;
    LR_speedSet = 0;

    /*----------------------------PID参数----------------------------*/
    M3508SpeedPid.kp = 200.0f;
    M3508SpeedPid.ki = 0.0f;
    M3508SpeedPid.kd = 10.0f;
    M3508SpeedPid.maxOut = 25000;
    M3508SpeedPid.maxIOut = 0;

    /*----------------------------右前轮----------------------------*/
    R_Front.controlMode = DJIMotor::SPD_MODE;
    R_Front.gearBox = GearBox::GearBox_M3508;
    R_Front.speedPid = M3508SpeedPid;
    R_Front.positionSet = R_Front.motorFeedback.positionFdb;
    R_Front.positionPid.Clear();
    R_Front.speedPid.Clear();
    R_Front.setOutput();
    R_Front.currentSet = 0;
    /*----------------------------左前轮----------------------------*/
    L_Front.controlMode = DJIMotor::SPD_MODE;
    L_Front.gearBox = GearBox::GearBox_M3508;
    L_Front.speedPid = M3508SpeedPid;
    L_Front.positionSet = L_Front.motorFeedback.positionFdb;
    L_Front.positionPid.Clear();
    L_Front.speedPid.Clear();
    L_Front.setOutput();
    L_Front.currentSet = 0;
    /*----------------------------右后轮----------------------------*/
    R_Rear.controlMode = DJIMotor::SPD_MODE;
    R_Rear.gearBox = GearBox::GearBox_M3508;
    R_Rear.speedPid = M3508SpeedPid;
    R_Rear.positionSet = R_Rear.motorFeedback.positionFdb;
    R_Rear.positionPid.Clear();
    R_Rear.speedPid.Clear();
    R_Rear.setOutput();
    R_Rear.currentSet = 0;
    /*----------------------------左后轮----------------------------*/
    L_Rear.controlMode = DJIMotor::SPD_MODE;
    L_Rear.gearBox = GearBox::GearBox_M3508;
    L_Rear.speedPid = M3508SpeedPid;
    L_Rear.positionSet = L_Rear.motorFeedback.positionFdb;
    L_Rear.positionPid.Clear();
    L_Rear.speedPid.Clear();
    L_Rear.setOutput();
    L_Rear.currentSet = 0;

    /*---------------------------挂载电机---------------------------*/
    // @TODO: Please ensure the motor IDs are consistent with chassis model
    DJIMotorHandler::Instance()->registerMotor(&L_Rear, &hcan1, 0x202);
    DJIMotorHandler::Instance()->registerMotor(&L_Front, &hcan1, 0x201);
    DJIMotorHandler::Instance()->registerMotor(&R_Front, &hcan1, 0x204);
    DJIMotorHandler::Instance()->registerMotor(&R_Rear, &hcan1, 0x203);
}

void ChassisController::Run()
{
    HandleInput();
    Kinematic_Inverse_Resolution(motors);
}
static uint32_t transmit_count = 0;
static uint32_t err_count = 0;
void ChassisController::HandleInput()
{
    /*
     * @TODO: implement your remote control logic here
     *    please use Dr16 class
     *    Vx =
     *    Vy =
     *    Vw =
     *    you have to determine your state according to remote control switch state
     */

    if (isnan(Vx) || isnan(Vy) || isnan(Vw)) // 如果出现nan错误，将速度设定值设为0
    {
        Vx = 0;
        Vy = 0;
        Vw = 0;
    }

    // // 一阶卡尔曼滤波
    // Vx = VxFilter.Update(Vx);
    // Vy = VyFilter.Update(Vy);
    // Vw = VwFilter.Update(Vw);
}

void ChassisController::Kinematic_Inverse_Resolution(M3508 *motors[])
{
    float tmp_Speed;
    for (uint8_t i = 0; i < 4; i++)
    {
        /*
         * @TODO: implement your IK solution here, the core part contains a single line
         *   tmp_Speed =
         *   motors[i]->speedSet =
         *   Please ensure that your controller works and your control signals are sent
         */
    }
}
