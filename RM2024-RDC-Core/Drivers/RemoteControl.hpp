#pragma once
#include "AppConfig.h"

#if USE_REMOTE_CONTROL
#include "FreeRTOS.h"
#include "task.h"

namespace RemoteControl{

#define NORMAL 1 // 正常移动 sw上
#define ADJUST 2 // 用于对位 sw下

//待调参
#define chassis_Kp 100
#define chassis_Ki 0
#define chassis_Kd 0
#define chassis_max_iout 100
#define chassis_max_out 100

#define MAX_RPM 100

struct chassis_move
{
    uint8_t mode;

    float vm1; 
    float vm2;
    float vm3;
    float vm4;

    float set_vx; // 轮向速度（正数前进，负数后退）
    float set_vy; // 垂直于轮向速度（正数向右，负数向左）
    float set_wz; // 旋转角速度（正数顺时针，负数逆时针）

    int16_t set_m1_current;
    int16_t set_m2_current;
    int16_t set_m3_current;
    int16_t set_m4_current;
};

/**
 * @brief update mode / set_vx / set_vy / set_wz from DR16 (percentage)
 * @retval none
 */
void classis_set_update();

/**
 * @brief calculate the target speeds of four wheels (rpm)
 * @retval none
 */
void forward_kinematics();

/**
 * @brief calculate the target current of each motor by PID
 * @retval none
 */
void classis_pid_calculate();

}
#endif