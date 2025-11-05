#ifndef CHASSIS_STATE_REMOTE_CONTROL_HPP
#define CHASSIS_STATE_REMOTE_CONTROL_HPP

#include "main.h"
#include "arm_math.h"
#include "string.h"
#include <cmath>

#include "M3508.hpp"
#include "DJIMotorHandler.hpp"

#include "StateMachine.hpp"

#include "filter.hpp"
#include "math.hpp"
#include "pid.hpp"

#define square(x) ((x) * (x))
using namespace Filter;
/**
 * @class ChassisController
 * @brief 底盘控制类，提供底盘的基本控制功能。
 * 用于底盘的C板，接收指令并控制底盘电机。
 */
class ChassisController : public StateMachine
{
public:
    /*电机实例*/
    M3508 R_Front; ///< 右前轮
    M3508 L_Front; ///< 左前轮
    M3508 R_Rear;  ///< 右后轮
    M3508 L_Rear;  ///< 左后轮

    /*用于储存四个底盘轮子的指针*/
    M3508 *motors[4] = {&R_Front, &L_Front, &L_Rear, &R_Rear};

    /*用于速度解算的变量，单位为 m/s。这些变量也是直接用于操控底盘移动的变量。*/
    float Vx;            ///< 绝对横向移动速度
    float Vy;            ///< 绝对前后移动速度
    float Vw;            ///< 绝对旋转速度
    float RelativeAngle; ///< 云台和底盘的相对角度

    float Fx[4];
    float Fy[4];
    float F[4];

    float Currents;

    /*速度解算后四个轮子的速度*/
    float RF_speedSet; ///< 右前轮速度设定
    float LF_speedSet; ///< 左前轮速度设定
    float RR_speedSet; ///< 右后轮速度设定
    float LR_speedSet; ///< 左后轮速度设定

    /*与速度解算相关的一些常量*/
    const float distanceToFrontAxle = 1.0f; ///< 中心到前轴距离
    const float distanceToRearAxle = 1.0f;  ///< 中心到后轴距离
    const float distanceToRightSide = 1.0f; ///< 中心到右侧距离
    const float distanceToLeftSide = 1.0f;  ///< 中心到左侧距离
    const float wheelRadius = 0.036f;        ///< 轮子半径
    const float wheel_resistance_threshold_speed = 1.0f;
		
    bool uart_received = false;
    uint8_t idle_count = 0;

    PID M3508SpeedPid = PID(0.1f, 0.0f, 0.0f, 25000.0f, 3.0f, PID_POSITION); ///< 速度环PID ;

    // KalmanFilter VxFilter; ///< 横向速度滤波器
    // KalmanFilter VyFilter; ///< 纵向速度滤波器
    // KalmanFilter VwFilter; ///< 旋转速度滤波器

    /**
     * @brief 初始化函数，用于挂载四个底盘电机，并设置PID参数
     */
    void Init() override;

    /**
     * @brief 运行函数，依次执行enter、Execute、exit函数
     */
    void Run() override;
    /**
     * @brief 构造函数，将位于controller的四个底盘电机指针传入
     */
    void HandleInput() override;

    void Kinematic_Inverse_Resolution(M3508* motors[]);

    ChassisController() {};
    ~ChassisController() {};

    static ChassisController *Instance()
    {
        static ChassisController instance;
        return &instance;
    }
};

#endif // CHASSIS_CONTROLLER_HPP
