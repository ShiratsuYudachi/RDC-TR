#include "PID.hpp"
#if USE_PID
namespace Control
{

#define LimitMax(input, max)   \
    {                          \
        if (input > max)       \
        {                      \
            input = max;       \
        }                      \
        else if (input < -max) \
        {                      \
            input = -max;      \
        }                      \
    }

/*This is where you implement the PID algorithm*/
float PID::update(float target, float measurement, float dt)
{
    error = target - measurement;

    pOut = Kp * error;
    iOut += Ki * error * dt;
    LimitMax(iOut, max_iout);
    dOut = Kd * (error - lastError) / dt;

    lastError = error;

    output = pOut + iOut + dOut;
    LimitMax(output, max_out);

    return this->output;
}

}  // namespace Control
#endif
