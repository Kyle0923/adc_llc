#include <ros/ros.h>
#include <cmath>
#include <stdexcept>
#include <pigpiod_if2.h>
#include "pca9685_if.hpp"

//credit: https://github.com/Reinbert/pca9685/blob/master/src/pca9685.c

static constexpr uint8_t PCA9685_I2C_ADDRESS = 0x40;

// Setup registers
static constexpr uint8_t PCA9685_MODE1    = 0x00;
static constexpr uint8_t PCA9685_PRESCALE = 0xFE;
static constexpr uint8_t PWM0_ON_L        = 0x06;
static constexpr uint8_t PWMALL_ON_L      = 0xFA;
static constexpr uint8_t PIN_ALL          = 16U;

constexpr uint8_t Pca9685IF::pwmRegister(const uint8_t pin)
{
    return (pin >= PIN_ALL ? PWMALL_ON_L : PWM0_ON_L + 4 * pin);
}

int Pca9685IF::setLeftPwm(const uint32_t aPwm)
{
    static constexpr uint8_t registerLeft = pwmRegister(LEFT_PWM_PIN);
    return setPwm(aPwm, registerLeft);
}

int Pca9685IF::setRightPwm(const uint32_t aPwm)
{
    static constexpr uint8_t registerRight = pwmRegister(RIGHT_PWM_PIN);
    return setPwm(aPwm, registerRight);
}

int Pca9685IF::setPwm(const uint32_t aPwm, const uint8_t aRegister)
{
    int status = 0;
    if (0U < aPwm && aPwm < 4096U)
    {
        status |= i2c_write_byte_data(mPiHandle, mI2CHandle, aRegister, 0U);
        status |= i2c_write_byte_data(mPiHandle, mI2CHandle, aRegister + 1U, 0U);
        status |= i2c_write_byte_data(mPiHandle, mI2CHandle, aRegister + 2U, static_cast<uint8_t>(aPwm & 0xFF));
        status |= i2c_write_byte_data(mPiHandle, mI2CHandle, aRegister + 3U, static_cast<uint8_t>((aPwm >> 8) & 0x0F));
    }
    else if (aPwm == 0)
    {
        status |= i2c_write_byte_data(mPiHandle, mI2CHandle, aRegister, 0U);
        status |= i2c_write_byte_data(mPiHandle, mI2CHandle, aRegister + 1U, 0U);
        status |= i2c_write_byte_data(mPiHandle, mI2CHandle, aRegister + 2U, 0U);
        status |= i2c_write_byte_data(mPiHandle, mI2CHandle, aRegister + 3U, 0x10); //bit 4, full off
    }
    else
    {
        status |= i2c_write_byte_data(mPiHandle, mI2CHandle, aRegister, 0U);
        status |= i2c_write_byte_data(mPiHandle, mI2CHandle, aRegister + 1U, 0x10); //bit 4, full on
        status |= i2c_write_byte_data(mPiHandle, mI2CHandle, aRegister + 2U, 0U);
        status |= i2c_write_byte_data(mPiHandle, mI2CHandle, aRegister + 3U, 0U);
    }
    return status;
}

int Pca9685IF::setPinFullOn(const uint8_t pin)
{
    int status = 0;
    const uint8_t lRegister = pwmRegister(pin);
    status |= i2c_write_byte_data(mPiHandle, mI2CHandle, lRegister, 0U);
    status |= i2c_write_byte_data(mPiHandle, mI2CHandle, lRegister + 1U, 0x10); //bit 4, full on
    status |= i2c_write_byte_data(mPiHandle, mI2CHandle, lRegister + 2U, 0U);
    status |= i2c_write_byte_data(mPiHandle, mI2CHandle, lRegister + 3U, 0U);
    return status;
}

int Pca9685IF::setPinFullOff(const uint8_t pin)
{
    int status = 0;
    const uint8_t lRegister = pwmRegister(pin);
    status |= i2c_write_byte_data(mPiHandle, mI2CHandle, lRegister, 0U);
    status |= i2c_write_byte_data(mPiHandle, mI2CHandle, lRegister + 1U, 0U);
    status |= i2c_write_byte_data(mPiHandle, mI2CHandle, lRegister + 2U, 0U);
    status |= i2c_write_byte_data(mPiHandle, mI2CHandle, lRegister + 3U, 0x10); //bit 4, full off
    return status;
}

int Pca9685IF::setDigital(const uint8_t pin, const uint8_t value)
{
    if (value == 0)
    {
        return setPinFullOff(pin);
    }
    else
    {
        return setPinFullOn(pin);
    }
}

void Pca9685IF::initPca9685()
{
    setPinFullOff(PIN_ALL);
    // To set pwm frequency we have to set the prescale register. The formula is:
    // prescale = round(osc_clock / (4096 * frequency))) - 1 where osc_clock = 25 MHz
    // frequency = [24...1526]
    // Further info here: http://www.nxp.com/documents/data_sheet/PCA9685.pdf Page 24
    const uint8_t prescale = static_cast<uint8_t>(25000000.0f / (4096.0f * PCA9685_FREQ) - 0.5f);

    // Get settings and calc bytes for the different states.
    const uint8_t settings = static_cast<uint8_t>(i2c_read_byte_data(mPiHandle, mI2CHandle, PCA9685_MODE1)) & uint8_t{0x7F};   // Set restart bit to 0
    const uint8_t sleep    = settings | uint8_t{0x10};          // Set sleep bit to 1
    const uint8_t wake     = settings & uint8_t{0xEF};          // Set sleep bit to 0
    const uint8_t restart  = wake | uint8_t{0x80};              // Set restart bit to 1

    // Go to sleep, set prescale and wake up again.
    i2c_write_byte_data(mPiHandle, mI2CHandle, PCA9685_MODE1, sleep); // sleep
    i2c_write_byte_data(mPiHandle, mI2CHandle, PCA9685_PRESCALE, prescale); // set prescale
    i2c_write_byte_data(mPiHandle, mI2CHandle, PCA9685_MODE1, wake); // wake

    // Now wait a millisecond until oscillator finished stabilizing and restart PWM.
    ros::Duration(1e-3).sleep();
    i2c_write_byte_data(mPiHandle, mI2CHandle, PCA9685_MODE1, restart); // restart
}

Pca9685IF::Pca9685IF(const int piHandle) : mPiHandle(piHandle)
{
    mI2CHandle = i2c_open(mPiHandle, 1U, PCA9685_I2C_ADDRESS, 0);
    if (mI2CHandle < 0)
    {
        throw std::runtime_error("unable to open I2C");
    }
    initPca9685();
}

Pca9685IF::~Pca9685IF()
{
    setPinFullOff(PIN_ALL);
    i2c_close(mPiHandle, mI2CHandle);
}