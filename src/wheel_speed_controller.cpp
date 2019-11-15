#include <ros/ros.h>
#include <cmath>
#include <pigpiod_if2.h>
#include <stdexcept>
#include "adc_llc.hpp"

#define SCALING_ENABLE true

RobotSpeeds WheelSpeedController::setWheelSpeed(const double v, const double w)
{
    if (v == 0 && w == 0)
    {
        setLeftDutyCycle(0);
        setRightDutyCycle(0);
        return RobotSpeeds{0, 0};
    }

    double leftRpm = getLeftWheelRpm(v, w);
    double rightRpm = getRightWheelRpm(v, w);

#if SCALING_ENABLE

    double leftRpmAbs = std::abs(leftRpm);
    double rightRpmAbs = std::abs(rightRpm);

    //scale up or down accordingly
    if (leftRpmAbs < LEFT_RPM_AT_6_VOLT || rightRpmAbs < RIGHT_RPM_AT_6_VOLT)
    {
        const double scalingFactor = std::max(LEFT_RPM_AT_6_VOLT / leftRpmAbs, RIGHT_RPM_AT_6_VOLT / rightRpmAbs);
        leftRpm *= scalingFactor;
        rightRpm *= scalingFactor;
    }
    else if (leftRpmAbs > LEFT_RPM_AT_12_VOLT || rightRpmAbs > RIGHT_RPM_AT_12_VOLT)
    {
        const double scalingFactor = std::min(LEFT_RPM_AT_12_VOLT / leftRpmAbs, RIGHT_RPM_AT_12_VOLT / rightRpmAbs);
        leftRpm *= scalingFactor;
        rightRpm *= scalingFactor;
    }

#endif

    int status = 0;
    const double leftDutyCycle = rpmToDutyCylce(leftRpm, \
                                                getRpmToVoltGradient(LEFT_RPM_AT_6_VOLT, LEFT_RPM_AT_12_VOLT), \
                                                getRpmToVoltBias(LEFT_RPM_AT_6_VOLT, LEFT_RPM_AT_12_VOLT));
    const double rightDutyCycle = rpmToDutyCylce(rightRpm, \
                                                getRpmToVoltGradient(RIGHT_RPM_AT_6_VOLT, RIGHT_RPM_AT_12_VOLT), \
                                                getRpmToVoltBias(RIGHT_RPM_AT_6_VOLT, RIGHT_RPM_AT_12_VOLT));
    status |= setLeftDutyCycle(leftDutyCycle);
    status |= setRightDutyCycle(rightDutyCycle);
    ////////// debug only
    // status |= setLeftDutyCycle(100 * rightDutyCycle / abs(rightDutyCycle));
    // status |= setRightDutyCycle(100 * rightDutyCycle / abs(rightDutyCycle));
    //////////
    ROS_INFO("LEFT Duty Cycle:%f", leftDutyCycle);
    ROS_INFO("RIGHT Duty Cycle:%f", rightDutyCycle);
    if (status == 0)
    {
        const double wL = leftRpm * 2.0 * PI / 60.0;
        const double wR = rightRpm * 2.0 * PI / 60.0;
        const double actualV = (wR + wL) * WHEEL_RADIUS / 2.0;
        const double actualW = (wR - wL) * WHEEL_RADIUS / WHEEL_DISTANCE;
        return RobotSpeeds{actualV, actualW};
    }
    else
    {
        setWheelSpeed(0, 0);
        throw std::runtime_error("Set PWM failed");
    }
}

double WheelSpeedController::getLeftWheelRpm(const double v, const double w)
{
    const double angularSpeed = (v - w * WHEEL_DISTANCE / 2.0) / WHEEL_RADIUS;
    return 60.0 * angularSpeed / (2.0 * PI);
}

double WheelSpeedController::getRightWheelRpm(const double v, const double w)
{
    const double angularSpeed = (v + w * WHEEL_DISTANCE / 2.0) / WHEEL_RADIUS;
    return 60.0 * angularSpeed / (2.0 * PI);
}

double WheelSpeedController::rpmToDutyCylce(const double rpm, const double gradient, const double bias)
{
    const double wheelVoltage = std::abs(rpm) * gradient + bias;
    if (rpm >= 0)
    {
        return wheelVoltage / VOLTAGE_MAX;
    }
    else
    {
        return (-1.0) * wheelVoltage / VOLTAGE_MAX;
    }
}

constexpr double WheelSpeedController::getRpmToVoltGradient(const double rpmAt6V, const double rpmAt12V)
{
    return (6.0 / (rpmAt12V - rpmAt6V));
}

constexpr double WheelSpeedController::getRpmToVoltBias(const double rpmAt6V, const double rpmAt12V)
{
    return (12 - (6 * rpmAt12V) / (rpmAt12V - rpmAt6V));
}

int WheelSpeedController::setLeftPwm(uint32_t aPwm)
{
    return mPca9685.setLeftPwm(aPwm);
}

int WheelSpeedController::setRightPwm(uint32_t aPwm)
{
    return mPca9685.setRightPwm(aPwm);
}

int WheelSpeedController::setLeftDutyCycle(const double aDutyCycle)
{
    if (aDutyCycle > 0)
    {
        gpio_write(mPiHandle, LOGIC_IN_1, 0);
        gpio_write(mPiHandle, LOGIC_IN_2, 1);
    }
    else if (aDutyCycle < 0)
    {
        gpio_write(mPiHandle, LOGIC_IN_1, 1);
        gpio_write(mPiHandle, LOGIC_IN_2, 0);
    }
    else //aDutyCycle = 0
    {
        gpio_write(mPiHandle, LOGIC_IN_1, 0);
        gpio_write(mPiHandle, LOGIC_IN_2, 0);
    }
    return setLeftPwm(dutyCycleToPwm(aDutyCycle));
}

int WheelSpeedController::setRightDutyCycle(const double aDutyCycle)
{
    if (aDutyCycle > 0)
    {
        gpio_write(mPiHandle, LOGIC_IN_3, 0);
        gpio_write(mPiHandle, LOGIC_IN_4, 1);
    }
    else if (aDutyCycle < 0)
    {
        gpio_write(mPiHandle, LOGIC_IN_3, 1);
        gpio_write(mPiHandle, LOGIC_IN_4, 0);
    }
    else //aDutyCycle = 0
    {
        gpio_write(mPiHandle, LOGIC_IN_3, 0);
        gpio_write(mPiHandle, LOGIC_IN_4, 0);
    }
    return setRightPwm(dutyCycleToPwm(aDutyCycle));
}

uint32_t WheelSpeedController::dutyCycleToPwm(const double aDutyCycle)
{
    return static_cast<uint32_t>( std::floor(std::abs(aDutyCycle) / 100.0 * static_cast<double>(PWM_MAX_VALUE)) );
}

WheelSpeedController::WheelSpeedController() :
    mPiHandle(pigpio_start(nullptr, nullptr)), mPca9685(mPiHandle)
{
    // mPiHandle = pigpio_start(nullptr, nullptr);
    if (mPiHandle < 0)
    {
        throw std::runtime_error("unable to start pigpio");
    }

    int setPinStatus = 0;

    setPinStatus |= set_mode(mPiHandle, LOGIC_IN_1, PI_OUTPUT);  // set PIN 35 as output
    setPinStatus |= set_mode(mPiHandle, LOGIC_IN_2, PI_OUTPUT);  // set PIN 37 as output
    setPinStatus |= set_mode(mPiHandle, LOGIC_IN_3, PI_OUTPUT);  // set PIN 36 as output
    setPinStatus |= set_mode(mPiHandle, LOGIC_IN_4, PI_OUTPUT);  // set PIN 38 as output

    if (setPinStatus != 0)
    {
        throw std::runtime_error("gpio pin setup failed");
    }
}

WheelSpeedController::~WheelSpeedController()
{
    setLeftPwm(0);
    setRightPwm(0);
    gpio_write(mPiHandle, LOGIC_IN_1, 0);
    gpio_write(mPiHandle, LOGIC_IN_2, 0);
    gpio_write(mPiHandle, LOGIC_IN_3, 0);
    gpio_write(mPiHandle, LOGIC_IN_4, 0);
    pigpio_stop(mPiHandle);
}
