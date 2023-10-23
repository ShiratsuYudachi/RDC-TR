#pragma once
#include "AppConfig.h"

#if USE_DR16

#include "main.h"
#include "usart.h"

namespace DR16{

    struct DR16Data{
        struct {
            struct {
                /* stick data */
                uint16_t ch0 = 1024U;
                uint16_t ch1 = 1024U;
                uint16_t ch2 = 1024U;
                uint16_t ch3 = 1024U;
            } stick;
            struct {
                /* switch data */
                uint8_t s1 = 0U;
                uint8_t s2 = 0U;
                uint8_t last_s1 = 0U;
                uint8_t last_s2 = 0U;
            } sw;
            
            int16_t wheel = 0;
            bool isConnect = false;
        } rc;
    };

    /**
     * @brief Initialize DR16
    */
    void init(void);
    
    /**
     * @brief Start receiving data from DR16
    */
    void startReceiveRC(void);

    /**
     * @brief Get DR16 data ptr
     * @return ptr to static DR16Data
    */
    DR16Data* getDR16Data(void);

    /**
     * @brief Update time to current time
    */
    void updateCurrentTime(void);

    /**
     * @brief Reset timeout
    */
    void resetTimeout(void);

    /**
     * @brief Reset DR16 data
    */
    void resetDR16Data(void);

    /**
     * @brief Check connection
    */
    void connectionCheck(void);

    /**
     * @brief Decode data from DR16
     * @param buffer: data buffer
    */
    void decodeData(uint8_t *buffer);

    /**
     * @brief Callback function for DR16 UART receive complete
     * @param huart: UART handler
     * @param size: size of received data
    */
    void DR16CompleteCallback(UART_HandleTypeDef *huart, uint16_t size);

    /**
     * @brief Callback function for DR16 UART error
     * @param huart: UART handler
    */
    void DR16ErrorCallback(UART_HandleTypeDef *huart);

    /**
     * @brief Adjust the format of output data
     * @param data: DR16Data
    */
    void formatOutput(DR16::DR16Data *data);

    /**
     * @brief Callback function for UART DMA Transmit complete
     * @param huart: UART handler
    */
    void HAL_UART_DMATransmitCpltCallback(UART_HandleTypeDef *huart);

}

#endif  // DR16_UART