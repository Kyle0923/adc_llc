#include <ros/ros.h>
#include <cmath>
#include <pigpiod_if2.h>
#include <stdexcept>
#include "adc_llc.hpp"

#define SCALING_ENABLE true

int WheelSpeedController::setWheelSpeed(const double v, const double w)
{
    int status = 0;
    if (v == 0 && w == 0)
    {
        status |= setLeftDutyCycle(0);
        status |= setRightDutyCycle(0);
        return status;
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
        mWL = leftRpm * 2.0 * PI / 60.0;
        mWR = rightRpm * 2.0 * PI / 60.0;
        ROS_INFO("wL[%f], wR[%f]", mWL, mWR);
        return status;
    }
    else
    {
        ROS_WARN("Set PWM failed");
        return setWheelSpeed(0, 0);
    }
}

RobotDisplacement WheelSpeedController::getRobotDisplacement()
{
    double angDispL = mpLeftEncoder->getAngularDisplacement();
    double angDispR = mpRightEncoder->getAngularDisplacement();
    if (mWL < 0)
    {
        angDispL = -angDispL;
    }
    if (mWR < 0)
    {
        angDispR = -angDispR;
    }
    const double bodyLinDisp = (angDispR + angDispL) * WHEEL_RADIUS / 2.0;
    //below, magic number 2.23, base on expriment that when dutyCycle = 1 & -1, rotation is about 2 rad, while reading only gives 0.93
    const double bodyAngDisp = (angDispR - angDispL) * WHEEL_RADIUS / WHEEL_DISTANCE * 2.23;
    return RobotDisplacement{bodyLinDisp, bodyAngDisp};
}

double WheelSpeedController::getLeftWheelRpm(const double v, const double w)
{
    /* (w / 40.0 * LEFT_RPM_AT_VOLT_MAX) base on expriment, when duty = -1 & 1,
     * rotation displacement is only 2 rad (corresponding to 40 rpm)*/
    const double angularSpeed = (v - w / 40.0 * LEFT_RPM_AT_VOLT_MAX * WHEEL_DISTANCE / 2.0) / WHEEL_RADIUS;
    return 60.0 * angularSpeed / (2.0 * PI);
}

double WheelSpeedController::getRightWheelRpm(const double v, const double w)
{
    const double angularSpeed = (v + w / 40.0 * LEFT_RPM_AT_VOLT_MAX * WHEEL_DISTANCE / 2.0) / WHEEL_RADIUS;
    return 60.0 * angularSpeed / (2.0 * PI);
}

double WheelSpeedController::rpmToDutyCylce(const double rpm, const double gradient, const double bias)
{
    const double wheelVoltage = std::abs(rpm) * gradient + bias;
    if (rpm >= 0)
    {
        return wheelVoltage / BATTERY_VOLTAGE;
    }
    else
    {
        return (-1.0) * wheelVoltage / BATTERY_VOLTAGE;
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
    return mpPca9685->setLeftPwm(aPwm);
}

int WheelSpeedController::setRightPwm(uint32_t aPwm)
{
    return mpPca9685->setRightPwm(aPwm);
}

int WheelSpeedController::setLeftDutyCycle(const double aDutyCycle)
{
    if (aDutyCycle > 0)
    {
        mpPca9685->setDigital(H_BRIDGE_LOGIC_3, 1);
        mpPca9685->setDigital(H_BRIDGE_LOGIC_4, 0);
    }
    else if (aDutyCycle < 0)
    {
        mpPca9685->setDigital(H_BRIDGE_LOGIC_3, 0);
        mpPca9685->setDigital(H_BRIDGE_LOGIC_4, 1);
    }
    else //aDutyCycle = 0
    {
        mpPca9685->setDigital(H_BRIDGE_LOGIC_3, 0);
        mpPca9685->setDigital(H_BRIDGE_LOGIC_4, 0);
    }
    return setLeftPwm(dutyCycleToPwm(aDutyCycle));
}

int WheelSpeedController::setRightDutyCycle(const double aDutyCycle)
{
    if (aDutyCycle > 0)
    {
        mpPca9685->setDigital(H_BRIDGE_LOGIC_1, 1);
        mpPca9685->setDigital(H_BRIDGE_LOGIC_2, 0);
    }
    else if (aDutyCycle < 0)
    {
        mpPca9685->setDigital(H_BRIDGE_LOGIC_1, 0);
        mpPca9685->setDigital(H_BRIDGE_LOGIC_2, 1);
    }
    else //aDutyCycle = 0
    {
        mpPca9685->setDigital(H_BRIDGE_LOGIC_1, 0);
        mpPca9685->setDigital(H_BRIDGE_LOGIC_2, 0);
    }
    return setRightPwm(dutyCycleToPwm(aDutyCycle));
}

uint32_t WheelSpeedController::dutyCycleToPwm(const double aDutyCycle)
{
    return static_cast<uint32_t>( std::floor(std::abs(aDutyCycle) * static_cast<double>(PWM_MAX_VALUE)) );
}

void WheelSpeedController::updateEncoder(const unsigned user_gpio)
{
    if (user_gpio == LEFT_ENCODER)
    {
        mpLeftEncoder->incrementReading();
    }
    else if (user_gpio == RIGHT_ENCODER)
    {
        mpRightEncoder->incrementReading();
    }
}

WheelSpeedController::WheelSpeedController() :
    mPiHandle(pigpio_start(nullptr, nullptr))
{
    // mPiHandle = pigpio_start(nullptr, nullptr);
    if (mPiHandle < 0)
    {
        throw std::runtime_error("unable to start pigpio");
    }
    mpPca9685 = new Pca9685IF(mPiHandle);
    mpLeftEncoder = new Lm393IF(mPiHandle, LEFT_ENCODER);
    mpRightEncoder = new Lm393IF(mPiHandle, RIGHT_ENCODER);
}

WheelSpeedController::~WheelSpeedController()
{
    delete mpRightEncoder;
    delete mpLeftEncoder;
    delete mpPca9685;
    pigpio_stop(mPiHandle);
}

void lm393Callback(int pi, unsigned user_gpio, unsigned level, uint32_t tick)
{
    if (level != 2) // if not watchdog timeout
    {
        ROS_INFO("received lm393 reading on PIN[%d], level[%d], tick[%d]", user_gpio, level, tick);
        pgAdcController->updateEncoder(user_gpio);
    }
}

