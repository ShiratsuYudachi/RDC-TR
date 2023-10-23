#pragma once
#include "AppConfig.h"

#if USE_PID
#include "FreeRTOS.h"
#include "task.h"


namespace Control
{

class PID
{
   public:
    /**
     * @brief Delete the default constructor.
     */
    PID() = delete;

    /**
     * @brief Constructor function
     * @param Kp_ The P term of the PID
     * @param Ki_ The I term of the PID
     * @param Kd_ The D term of the PID
     */
    PID(float Kp_ = 0, float Ki_ = 0, float Kd_ = 0, float max_iout_ = 0, float max_out_ = 0)
        : Kp(Kp_), Ki(Ki_), Kd(Kd_), max_iout(max_iout_), max_out(max_out_){};
    /**
     * @brief Update the PID status and return
     * @note  You need to implement the specific definition in the PID.cpp file
     * @param target        The target value
     * @param measurement   The feedback value
     * @param dt            The time interval between two updates
     */
    float update(float target, float measurement, float dt = 0.001f);

   private:
    float Kp = 0;  // The P param of the PID
    float Ki = 0;  // The I param of the PID
    float Kd = 0;  // The D param of the PID

    float error     = 0;  // The error in this update
    float lastError = 0;  // The error from last update

    float pOut = 0; // The P term output of the PID
    float iOut = 0; // The I term output of the PID
    float dOut = 0; // The D term output of the PID

    float output = 0;  // The current output of the PID

    float max_out = 0;
    float max_iout = 0;

};
}  // namespace Control

#endif
