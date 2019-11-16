#ifndef ADC_LLC_HPP
#define ADC_LLC_HPP

#include "pca9685_if.hpp"

struct RobotSpeeds
{
    double v;
    double w;
};

class WheelSpeedController
{
    public:
        RobotSpeeds setWheelSpeed(const double v, const double w);

        WheelSpeedController();
        ~WheelSpeedController();
        WheelSpeedController(const WheelSpeedController&) = delete;
        WheelSpeedController& operator=(const WheelSpeedController&) = delete;

    private:
        static constexpr double WHEEL_RADIUS   = 0.057; //m
        static constexpr double WHEEL_DISTANCE = 0.128; //m

        static constexpr double VOLTAGE_MAX    = 11.0;  //Volt
        static constexpr double VOLTAGE_MIN    = 7.0;  //Volt

        static constexpr double LEFT_RPM_AT_VOLT_MIN  = 21.78; //RPM
        static constexpr double LEFT_RPM_AT_VOLT_MAX  = 54.4; //RPM
        static constexpr double RIGHT_RPM_AT_VOLT_MIN = 21.78; //RPM
        static constexpr double RIGHT_RPM_AT_VOLT_MAX = 54.4;//RPM

        static constexpr double PI = 3.14159265359;
        static constexpr uint32_t PWM_MAX_VALUE     = 4096U; // 0~4096

        int mPiHandle;
        Pca9685IF mPca9685;
        int setLeftPwm(uint32_t aPwm);
        int setRightPwm(uint32_t aPwm);
        int setLeftDutyCycle(const double aDutyCycle);
        int setRightDutyCycle(const double aDutyCycle);
        uint32_t dutyCycleToPwm(const double aDutyCycle);
        double getLeftWheelRpm(const double v, const double w);
        double getRightWheelRpm(const double v, const double w);
        double rpmToDutyCylce(const double rpm, const double gradient, const double bias);
        constexpr double getRpmToVoltGradient(const double rpmAtVmin, const double rpmAtVmax);
        constexpr double getRpmToVoltBias(const double rpmAtVmin, const double rpmAtVmax);
};

#endif //ADC_LLC_HPP