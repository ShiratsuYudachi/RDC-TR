#pragma once
#include "AppConfig.h"

#if USE_DJI_MOTOR

#ifndef DJI_MOTOR_CAN
#define DJI_MOTOR_CAN hcan
#endif

#include "main.h"
#include "can.h"


namespace DJIMotor
{
enum Motor_ID
{
    CAN_CHASSIS_ALL_ID = 0x200,
    CAN_3508_M1_ID = 0x201,
    CAN_3508_M2_ID = 0x202,
    CAN_3508_M3_ID = 0x203,
    CAN_3508_M4_ID = 0x204,

    CAN_UPPER_ALL_ID = 0x1FF,
    CAN_GRIPPING_MOTOR_ID = 0x205,
    CAN_LIFTING_MOTOR_ID = 0x206,
};

struct DJIMotor
{
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperate;
    int16_t last_ecd;
};

/**
 * @brief The whole motor's module initialization function
 * @note  You might initialize the CAN Module here
 * @retval
 */
void init();

/**
  * @brief          hal CAN fifo call back, receive motor data
  * @param[in]      hcan, the point to CAN handle
  * @retval         none
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);

/**
  * @brief          send control current of motor (0x201, 0x202, 0x203, 0x204)
  * @param[in]      motor1: (0x201) 3508 motor control current, range [-16384,16384] 
  * @param[in]      motor2: (0x202) 3508 motor control current, range [-16384,16384] 
  * @param[in]      motor3: (0x203) 3508 motor control current, range [-16384,16384] 
  * @param[in]      motor4: (0x204) 3508 motor control current, range [-16384,16384] 
  * @retval         none
  */
void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);

/**
  * @brief          send control current of motor (0x205, 0x206)
  * @param[in]      motor1: (0x205) gripping motor control current, range [-16384,16384] 
  * @param[in]      motor2: (0x206) lifting motor control current, range [-16384,16384] 
  * @retval         none
  */
void CAN_cmd_upper(int16_t gripping_motor, int16_t lifting_motor);

/**
  * @brief Get DJIMotor data ptr
  * @return ptr to const DJIMotor data
  */
const DJIMotor *get_motor_measure_point(uint8_t i);

}  // namespace DJIMotor
#endif  // USE_DJI_MOTOR