#include <stdexcept>
#include <pigpiod_if2.h>
#include <ros/ros.h>
#include "lm393_if.hpp"

static constexpr double PI = 3.14159265359;

Lm393IF::Lm393IF(int aPiHandle, unsigned aPin) : mPin(aPin), mReading(0)
{
    if (0 != set_mode(aPiHandle, aPin, PI_INPUT))
    {
        ROS_WARN("unable to set PIN[%d] as input", aPin);
        throw std::runtime_error("unable to set PIN as input");
    }
    mCallbackID = callback(aPiHandle, aPin, EITHER_EDGE, lm393Callback);
}

double Lm393IF::getAngularDisplacement()
{
    const double angDisp = mReading * 2 * PI / READING_OF_FULL_REVOLUTION;
    mReading = 0;
    return angDisp;
}

uint32_t Lm393IF::incrementReading()
{
    return ++mReading;
}

Lm393IF::~Lm393IF()
{
    callback_cancel(mCallbackID);
}