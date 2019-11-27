#include <stdexcept>
#include <pigpiod_if2.h>
#include <ros/ros.h>
#include "lm393_if.hpp"

static constexpr double PI = 3.14159265359;

Lm393IF::Lm393IF(int aPiHandle, unsigned aPin) : mPiHandle(aPiHandle), mPin(aPin), mReading(0)
{
    if (0 != set_mode(mPiHandle, mPin, PI_INPUT))
    {
        ROS_WARN("unable to set PIN[%d] as input", aPin);
        throw std::runtime_error("unable to set PIN as input");
    }
    if (0 != set_watchdog(mPiHandle, mPin, 1000U))
    {
        ROS_WARN("unable to set watch dog on PIN[%d]", aPin);
        throw std::runtime_error("set watch dog failed");
    }
    mCallbackID = callback(mPiHandle, mPin, EITHER_EDGE, lm393Callback);
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
    set_watchdog(mPiHandle, mPin, 0U);
    callback_cancel(mCallbackID);
}