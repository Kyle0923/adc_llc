#ifndef ADC_LLC_HPP
#define ADC_LLC_HPP

#include "pca9685_if.hpp"
#include "lm393_if.hpp"
#include "mpu6050_if.hpp"

struct RobotDisplacement
{
    double linear;
    double angular;
};

class WheelSpeedController
{
    public:
        int setWheelSpeed(const double v, const double w);

        RobotDisplacement getRobotDisplacement();
        void updateEncoder(const unsigned user_gpio);
        void updateGyro(const uint32_t tick);
        void disableGyro();

        WheelSpeedController();
        ~WheelSpeedController();
        WheelSpeedController(const WheelSpeedController&) = delete;
        WheelSpeedController& operator=(const WheelSpeedController&) = delete;

    private:
        static constexpr unsigned LEFT_ENCODER  = 26; //pin37
        static constexpr unsigned RIGHT_ENCODER = 20; //pin38

        static constexpr double WHEEL_RADIUS   = 0.033; //m
        static constexpr double WHEEL_DISTANCE = 0.129; //m

        static constexpr double VOLTAGE_MAX    = 8.0;  //Volt
        static constexpr double VOLTAGE_MIN    = 3.5;  //Volt
        static constexpr double BATTERY_VOLTAGE = 8.0; //Volt

        static constexpr double LEFT_RPM_AT_VOLT_MIN  = 93.9;  //RPM
        static constexpr double LEFT_RPM_AT_VOLT_MAX  = 242.5; //RPM
        static constexpr double RIGHT_RPM_AT_VOLT_MIN = 93.9;  //RPM
        static constexpr double RIGHT_RPM_AT_VOLT_MAX = 242.5; //RPM

        static constexpr double PI = 3.14159265359;
        static constexpr uint32_t PWM_MAX_VALUE     = 4096U; // 0~4096

        //ideal angular velocity
        double mWL;
        double mWR;

        int mPiHandle;
        Pca9685IF* mpPca9685;
        Lm393IF* mpLeftEncoder;
        Lm393IF* mpRightEncoder;
        Mpu6050IF* mpMpu6050;
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

extern WheelSpeedController* pgAdcController;

#endif //ADC_LLC_HPP