#pragma once
#ifndef TASKWHEEL_HPP
#define TASKWHEEL_HPP

#include "cstdint"
#include "tx_api.h"


#include "DJIMotorHandler.hpp"
#include "LKMotorHandler.hpp"
#include "LK9025.hpp"
#include "LK8016.hpp"
#include "GM6020.hpp"
#include "M3508.hpp"
#include "M2006.hpp"


class ServiceMotors
{
public:


    M3508 LFWheel;
    M3508 LRWheel;
    M3508 RFWheel;
    M3508 RRWheel;



    void MotorRegister();
    void AllMotorSetOutput();
    void SetModeAndPidParam();

    static ServiceMotors *Instance()
    {
        static ServiceMotors instance;
        return &instance;
    }
};


#endif