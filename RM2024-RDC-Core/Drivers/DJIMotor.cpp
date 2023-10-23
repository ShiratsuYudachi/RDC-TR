#include "DJIMotor.hpp"

#ifdef USE_DJI_MOTOR
#ifndef RDC_DJIMotor_MAX_NUM
#define RDC_DJIMotor_MAX_NUM 8
#endif

namespace DJIMotor
{

// Initialize motor's controller instance
DJIMotor motors[RDC_DJIMotor_MAX_NUM];

void init()
{
    CAN_FilterTypeDef can_filter;
    can_filter.FilterBank = 0;
    can_filter.FilterMode = CAN_FILTERMODE_IDLIST;
    can_filter.FilterScale = CAN_FILTERSCALE_16BIT;
    can_filter.FilterIdHigh = (CAN_CHASSIS_ALL_ID << 5) >> 16;
    can_filter.FilterIdLow = (CAN_CHASSIS_ALL_ID << 5) & 0xFFFF;
    can_filter.FilterMaskIdHigh = (0xFF8 << 5) >> 16;
    can_filter.FilterMaskIdLow = (0xFF8 << 5) & 0xFFFF;
    can_filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    can_filter.FilterActivation = ENABLE;
    //掩码模式，只接受0x200-0x207
    
    HAL_CAN_ConfigFilter(&DJI_MOTOR_CAN, &can_filter);
    HAL_CAN_Start(&DJI_MOTOR_CAN);
    HAL_CAN_ActivateNotification(&DJI_MOTOR_CAN, CAN_IT_RX_FIFO0_MSG_PENDING);
    //FIFO0消息挂起中断,其回调函数是HAL_CAN_RxFifo0MsgPendingCallback
}

#define get_motor_measure(ptr, data)                                    \
    {                                                                   \
        (ptr)->last_ecd = (ptr)->ecd;                                   \
        (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);            \
        (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);      \
        (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);  \
        (ptr)->temperate = (data)[6];                                   \
    }

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);

    switch (rx_header.StdId)
    {
        case CAN_3508_M1_ID:
        case CAN_3508_M2_ID:
        case CAN_3508_M3_ID:
        case CAN_3508_M4_ID:
        case CAN_GRIPPING_MOTOR_ID:
        case CAN_LIFTING_MOTOR_ID:
        {
            static uint8_t i = 0;
            //get motor id
            i = rx_header.StdId - CAN_3508_M1_ID;
            get_motor_measure(&motors[i], rx_data);
            break;
        }
        default:
        {
            break;
        }
    }
}

static CAN_TxHeaderTypeDef chassis_tx_message;
static uint8_t chassis_can_send_data[8] = {0};
void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    uint32_t send_mail_box;
    chassis_tx_message.StdId = CAN_CHASSIS_ALL_ID;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    chassis_can_send_data[0] = motor1 >> 8;
    chassis_can_send_data[1] = motor1;
    chassis_can_send_data[2] = motor2 >> 8;
    chassis_can_send_data[3] = motor2;
    chassis_can_send_data[4] = motor3 >> 8;
    chassis_can_send_data[5] = motor3;
    chassis_can_send_data[6] = motor4 >> 8;
    chassis_can_send_data[7] = motor4;

    HAL_CAN_AddTxMessage(&DJI_MOTOR_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}

static CAN_TxHeaderTypeDef upper_tx_message;
static uint8_t upper_can_send_data[8] = {0};
void CAN_cmd_upper(int16_t gripping_motor, int16_t lifting_motor)
{
    uint32_t send_mail_box;
    upper_tx_message.StdId = CAN_UPPER_ALL_ID;
    upper_tx_message.IDE = CAN_ID_STD;
    upper_tx_message.RTR = CAN_RTR_DATA;
    upper_tx_message.DLC = 0x08;
    upper_can_send_data[0] = gripping_motor >> 8;
    upper_can_send_data[1] = gripping_motor;
    upper_can_send_data[2] = lifting_motor >> 8;
    upper_can_send_data[3] = lifting_motor;

    HAL_CAN_AddTxMessage(&DJI_MOTOR_CAN, &upper_tx_message, upper_can_send_data, &send_mail_box);
}

const DJIMotor *get_motor_measure_point(uint8_t i)
{
    return &motors[i - CAN_3508_M1_ID];
}

}  // namespace DJIMotor
#endif