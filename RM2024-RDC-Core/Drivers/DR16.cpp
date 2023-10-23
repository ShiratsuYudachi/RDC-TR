#include "DR16.hpp"

#if USE_DR16

#include "main.h"
#include "stdio.h"

namespace DR16{

#define MAX_TIMEOUT 25
#define DEAD_ZONE 5

// DR16Data obj & ptr
static DR16Data rc_data;
static DR16Data *rc_data_ptr = &rc_data;
// UART Rx buffer
static uint8_t rc_buffer[2][18];
// Control info & flags
static uint64_t signal_received = 0;
static uint64_t signal_lost = 0;
static uint64_t currTime = 0;
static uint64_t connection_timeout = 0;

void init(void){
    resetDR16Data();
    HAL_UART_RegisterRxEventCallback(&DR16_UART, DR16CompleteCallback);
    HAL_UART_RegisterCallback(&DR16_UART, HAL_UART_ERROR_CB_ID, DR16ErrorCallback);

    return;
}

void startReceiveRC(){
    HAL_UARTEx_ReceiveToIdle_IT(&huart1, (uint8_t*)rc_buffer, 18);

    return;
}

DR16Data *getDR16Data(void){

    return rc_data_ptr;
}

void updateCurrentTime(void){
    currTime = HAL_GetTick();

    return;
}

void resetTimeout(void){
    connection_timeout = HAL_GetTick() + MAX_TIMEOUT;

    return;
}

void resetDR16Data(void){
    rc_data.rc.stick.ch0 = 1024U;
    rc_data.rc.stick.ch1 = 1024U;
    rc_data.rc.stick.ch2 = 1024U;
    rc_data.rc.stick.ch3 = 1024U;
    rc_data.rc.sw.s1 = 0;
    rc_data.rc.sw.s2 = 0;
    rc_data.rc.wheel = 0;
    rc_data.rc.isConnect = false;

    return;
}

void connectionCheck(void){
    if (HAL_GetTick() > connection_timeout){
        resetDR16Data();
        rc_data.rc.isConnect = false;
    } else {
        rc_data.rc.isConnect = true;
    }
    updateCurrentTime();

    return;
}

void decodeData(uint8_t *buffer){
    // stick
    if (uint16_t ch0 = (uint16_t)(buffer[0] | (buffer[1] << 8)) & 0x07FF; 
        (ch0 > 1024 + DEAD_ZONE || ch0 < 1024 - DEAD_ZONE) && 
        (ch0 <= 1684 && ch0 >= 364)) {
        rc_data.rc.stick.ch0 = ch0;
    } else {
        rc_data.rc.stick.ch0 = 1024;
    }

    if (uint16_t ch1 = ((uint16_t)(buffer[1] >> 3) | ((uint16_t)buffer[2] << 5)) & 0x07FF; 
        (ch1 > 1024 + DEAD_ZONE || ch1 < 1024 - DEAD_ZONE) && 
        (ch1 <= 1684 && ch1 >= 364)) {
        rc_data.rc.stick.ch1 = ch1;
    } else {
        rc_data.rc.stick.ch1 = 1024;
    }

    if (uint16_t ch2 = ((uint16_t)(buffer[2] >> 6) | ((uint16_t)buffer[3] << 2) | ((uint16_t)buffer[4] << 10)) & 0x07FF; 
        (ch2 > 1024 + DEAD_ZONE || ch2 < 1024 - DEAD_ZONE) && 
        (ch2 <= 1684 && ch2 >= 364)) {
        rc_data.rc.stick.ch2 = ch2;
    } else {
        rc_data.rc.stick.ch2 = 1024;
    }

    if (uint16_t ch3 = ((uint16_t)(buffer[4] >> 1) | ((uint16_t)buffer[5] << 7)) & 0x07FF; 
        (ch3 > 1024 + DEAD_ZONE || ch3 < 1024 - DEAD_ZONE) && 
        (ch3 <= 1684 && ch3 >= 364)) {
        rc_data.rc.stick.ch3 = ch3;
    } else {
        rc_data.rc.stick.ch3 = 1024;
    }
    // switch
    rc_data.rc.sw.s1 = ((buffer[5] >> 4) & 0x000C) >> 2;
    rc_data.rc.sw.s2 = ((buffer[5] >> 4) & 0x0003);
    // wheel
    rc_data.rc.wheel = (int16_t)(buffer[16] | (buffer[17] << 8));

    return;
}

void DR16CompleteCallback(UART_HandleTypeDef *huart, uint16_t size){
    if (size == 18){
        decodeData((uint8_t*)rc_buffer);
        HAL_UARTEx_ReceiveToIdle_IT(&DR16_UART, (uint8_t*)rc_buffer, 18);
        signal_received++;
    } else {
        DR16ErrorCallback(&huart1);
    }
    resetTimeout();

    return;
}

void DR16ErrorCallback(UART_HandleTypeDef *huart){
    HAL_UART_Abort(&huart1);
    HAL_UARTEx_ReceiveToIdle_IT(&huart1, (uint8_t*)rc_buffer, 18);
    signal_lost++;

    return;
}

uint16_t txBuffer[95] = {0};

void formatOutput(DR16::DR16Data *data){
    sprintf((char *)txBuffer, 
                "ch0: %-4d, ch1: %-5d, ch2: %-4d, ch3: %-4d, s1: %-1d, s2: %-1d, wheel: %-4d, isConnect: %-1d\r\n", 
                data->rc.stick.ch0, data->rc.stick.ch1, data->rc.stick.ch2, data->rc.stick.ch3, 
                data->rc.sw.s1, data->rc.sw.s2, data->rc.wheel, data->rc.isConnect
            );
}

void HAL_UART_DMATransmitCpltCallback(UART_HandleTypeDef *huart){
    if (huart->Instance == USART3){
        formatOutput(DR16::getDR16Data());
        HAL_UART_Transmit_DMA(&huart3, (uint8_t *)txBuffer, 95);
    }
}

}  // namespace DR16

#endif