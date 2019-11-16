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
    if (leftRpmAbs < LEFT_RPM_AT_VOLT_MIN || rightRpmAbs < RIGHT_RPM_AT_VOLT_MIN)
    {
        const double scalingFactor = std::max(LEFT_RPM_AT_VOLT_MIN / leftRpmAbs, RIGHT_RPM_AT_VOLT_MIN / rightRpmAbs);
        leftRpm *= scalingFactor;
        rightRpm *= scalingFactor;
    }
    else if (leftRpmAbs > LEFT_RPM_AT_VOLT_MAX || rightRpmAbs > RIGHT_RPM_AT_VOLT_MAX)
    {
        const double scalingFactor = std::min(LEFT_RPM_AT_VOLT_MAX / leftRpmAbs, RIGHT_RPM_AT_VOLT_MAX / rightRpmAbs);
        leftRpm *= scalingFactor;
        rightRpm *= scalingFactor;
    }

#endif

    int status = 0;
    const double leftDutyCycle = rpmToDutyCylce(leftRpm, \
                                                getRpmToVoltGradient(LEFT_RPM_AT_VOLT_MIN, LEFT_RPM_AT_VOLT_MAX), \
                                                getRpmToVoltBias(LEFT_RPM_AT_VOLT_MIN, LEFT_RPM_AT_VOLT_MAX));
    const double rightDutyCycle = rpmToDutyCylce(rightRpm, \
                                                getRpmToVoltGradient(RIGHT_RPM_AT_VOLT_MIN, RIGHT_RPM_AT_VOLT_MAX), \
                                                getRpmToVoltBias(RIGHT_RPM_AT_VOLT_MIN, RIGHT_RPM_AT_VOLT_MAX));
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

constexpr double WheelSpeedController::getRpmToVoltGradient(const double rpmAtVmin, const double rpmAtVmax)
{
    return (VOLTAGE_MAX - VOLTAGE_MIN) / (rpmAtVmax - rpmAtVmin);
}

constexpr double WheelSpeedController::getRpmToVoltBias(const double rpmAtVmin, const double rpmAtVmax)
{
    return VOLTAGE_MAX - getRpmToVoltGradient(rpmAtVmin, rpmAtVmax) * rpmAtVmax;
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
        mPca9685.setDigital(H_BRIDGE_LOGIC_3, 0);
        mPca9685.setDigital(H_BRIDGE_LOGIC_4, 1);
    }
    else if (aDutyCycle < 0)
    {
        mPca9685.setDigital(H_BRIDGE_LOGIC_3, 1);
        mPca9685.setDigital(H_BRIDGE_LOGIC_4, 0);
    }
    else //aDutyCycle = 0
    {
        mPca9685.setDigital(H_BRIDGE_LOGIC_3, 0);
        mPca9685.setDigital(H_BRIDGE_LOGIC_4, 0);
    }
    return setLeftPwm(dutyCycleToPwm(aDutyCycle));
}

int WheelSpeedController::setRightDutyCycle(const double aDutyCycle)
{
    if (aDutyCycle > 0)
    {
        mPca9685.setDigital(H_BRIDGE_LOGIC_1, 0);
        mPca9685.setDigital(H_BRIDGE_LOGIC_2, 1);
    }
    else if (aDutyCycle < 0)
    {
        mPca9685.setDigital(H_BRIDGE_LOGIC_1, 1);
        mPca9685.setDigital(H_BRIDGE_LOGIC_2, 0);
    }
    else //aDutyCycle = 0
    {
        mPca9685.setDigital(H_BRIDGE_LOGIC_1, 0);
        mPca9685.setDigital(H_BRIDGE_LOGIC_2, 0);
    }
    return setRightPwm(dutyCycleToPwm(aDutyCycle));
}

uint32_t WheelSpeedController::dutyCycleToPwm(const double aDutyCycle)
{
    return static_cast<uint32_t>( std::floor(std::abs(aDutyCycle) * static_cast<double>(PWM_MAX_VALUE)) );
}

WheelSpeedController::WheelSpeedController() :
    mPiHandle(pigpio_start(nullptr, nullptr)), mPca9685(mPiHandle)
{
    // mPiHandle = pigpio_start(nullptr, nullptr);
    if (mPiHandle < 0)
    {
        throw std::runtime_error("unable to start pigpio");
    }
}

WheelSpeedController::~WheelSpeedController()
{
    setLeftPwm(0);
    setRightPwm(0);
    pigpio_stop(mPiHandle);
}
