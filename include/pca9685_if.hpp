#ifndef PCA9685_IF_HPP
#define PCA9685_IF_HPP

// PAC9685 PIN defination
static constexpr uint8_t RIGHT_PWM_PIN    = 0U;
static constexpr uint8_t H_BRIDGE_LOGIC_1 = 1U;
static constexpr uint8_t H_BRIDGE_LOGIC_2 = 2U;
static constexpr uint8_t H_BRIDGE_LOGIC_3 = 3U;
static constexpr uint8_t H_BRIDGE_LOGIC_4 = 4U;
static constexpr uint8_t LEFT_PWM_PIN     = 5U;

class Pca9685IF
{
    public:
        Pca9685IF(const int piHandle);
        Pca9685IF() = delete;
        ~Pca9685IF();
        int setLeftPwm(const uint32_t aPwm);
        int setRightPwm(const uint32_t aPwm);
        int setDigital(const uint8_t pin, const uint8_t value);

    private:
        int mPiHandle;
        int mI2CHandle;
        void initPca9685();
        static constexpr float PCA9685_FREQ = 500;
        static constexpr uint8_t pwmRegister(const uint8_t pin);
        int setPwm(const uint32_t aPwm, const uint8_t aRegister);
        int setPinFullOn(const uint8_t pin);
        int setPinFullOff(const uint8_t pin);
};

#endif // PCA9685_IF_HPP