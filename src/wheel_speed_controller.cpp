#include <ros/ros.h>
#include <cmath>
#include <pigpiod_if2.h>
#include <stdexcept>
#include "adc_llc.hpp"

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
    double leftRpmAbs = std::abs(leftRpm);
    double rightRpmAbs = std::abs(rightRpm);

    //scale up or down accordingly
    if (leftRpmAbs < RPM_MIN || rightRpmAbs < RPM_MIN)
    {
        const double scalingFactor = RPM_MIN / std::min(leftRpmAbs, rightRpmAbs);
        leftRpm *= scalingFactor;
        rightRpm *= scalingFactor;
    }
    else if (leftRpmAbs > RPM_MAX || rightRpmAbs > RPM_MAX)
    {
        const double scalingFactor = RPM_MAX / std::max(leftRpmAbs, rightRpmAbs);
        leftRpm *= scalingFactor;
        rightRpm *= scalingFactor;
    }

    int status = 0;
    status |= setLeftDutyCycle( rpmToDutyCylce(leftRpm) );
    status |= setRightDutyCycle( rpmToDutyCylce(rightRpm) );
    ROS_INFO("LEFT Duty Cycle:%f", rpmToDutyCylce(leftRpm));
    ROS_INFO("RIGHT Duty Cycle:%f", rpmToDutyCylce(rightRpm));
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

double WheelSpeedController::rpmToDutyCylce(const double rpm)
{
    const double wheelVoltage = std::abs(rpm) / 30.0 + 1.0 / 3.0;
    if (rpm >= 0)
    {
        return wheelVoltage * VOLTAGE_MAX;
    }
    else
    {
        return (-1.0) * wheelVoltage * VOLTAGE_MAX;
    }
}

int WheelSpeedController::setLeftPwm(uint32_t aPwm)
{
    ROS_INFO("L PWM:[%u]", aPwm);
    return hardware_PWM(mPiHandle, PWM_LEFT, PWM_FREQ, aPwm);
}

int WheelSpeedController::setRightPwm(uint32_t aPwm)
{
    ROS_INFO("L PWM:[%u]", aPwm);
    return hardware_PWM(mPiHandle, PWR_RIGHT, PWM_FREQ, aPwm);
}

int WheelSpeedController::setLeftDutyCycle(const double aDutyCycle)
{
    if (aDutyCycle > 0)
    {
        gpio_write(mPiHandle, LOGIC_IN_1, 1);
        gpio_write(mPiHandle, LOGIC_IN_2, 0);
    }
    else if (aDutyCycle < 0)
    {
        gpio_write(mPiHandle, LOGIC_IN_1, 0);
        gpio_write(mPiHandle, LOGIC_IN_2, 1);
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
        gpio_write(mPiHandle, LOGIC_IN_3, 1);
        gpio_write(mPiHandle, LOGIC_IN_4, 0);
    }
    else if (aDutyCycle < 0)
    {
        gpio_write(mPiHandle, LOGIC_IN_3, 0);
        gpio_write(mPiHandle, LOGIC_IN_4, 1);
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

WheelSpeedController::WheelSpeedController()
{
    mPiHandle = pigpio_start(nullptr, nullptr);
    if (mPiHandle < 0)
    {
        throw std::runtime_error("unable to start pigpio");
    }

    int setPinStatus = 0;
    setPinStatus |= setLeftPwm(PWM_DEFAULT_VALUE);  // set PIN 32 as PWM output
    setPinStatus |= setRightPwm(PWM_DEFAULT_VALUE); // set PIN 33 as PWM output

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
    pigpio_stop(mPiHandle);
}
