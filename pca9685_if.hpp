#ifndef PCA9685_IF_HPP
#define PCA9685_IF_HPP

class Pca9685IF
{
    public:
        Pca9685IF(const int piHandle);
        Pca9685IF() = delete;
        ~Pca9685IF();
        int setLeftPwm(const uint32_t aPwm);
        int setRightPwm(const uint32_t aPwm);

    private:
        int mPiHandle;
        unsigned mI2CHandle;
        void initPca9685();
        static constexpr float PCA9685_FREQ = 500;
        static constexpr uint8_t pwmRegister(const uint8_t pin);
        int setPwm(const uint32_t aPwm, const uint8_t aRegister);
};

#endif // PCA9685_IF_HPP