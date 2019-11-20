#include <stdexcept>
#include <pigpiod_if2.h>
#include <ros/ros.h>
#include "lm393_if.hpp"

Lm393IF::Lm393IF(int aPiHandle, unsigned aPin) : mPin(aPin), mReading(0)
{
    if (0 != set_mode(aPiHandle, aPin, PI_INPUT))
    {
        ROS_WARN("unable to set PIN[%d] as input", aPin);
        throw std::runtime_error("unable to set PIN as input");
    }
    mLevel = gpio_read(aPiHandle, aPin);
    callbackID = callback(aPiHandle, aPin, EITHER_EDGE, lm393Callback);
}

uint32_t Lm393IF::getReading()
{
    return mReading;
}

uint32_t Lm393IF::incrementReading()
{
    return ++mReading;
}