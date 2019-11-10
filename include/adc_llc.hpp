// PIN definition
constexpr unsigned PWM_LEFT  = 12;   // PIN 32
constexpr unsigned PWR_RIGHT = 13;   // PIN 33

constexpr unsigned LOGIC_IN_1 = 16;  // pin 36
constexpr unsigned LOGIC_IN_2 = 20;  // pin 38
constexpr unsigned LOGIC_IN_3 = 19;  // pin 35
constexpr unsigned LOGIC_IN_4 = 26;  // pin 37

struct RobotSpeeds
{
    double v;
    double w;
};

class WheelSpeedController
{
    public:
        WheelSpeedController();
        ~WheelSpeedController();
        RobotSpeeds setWheelSpeed(const double v, const double w);
        double getLeftWheelRpm(const double v, const double w);
        double getRightWheelRpm(const double v, const double w);
        double rpmToDutyCylce(const double rpm);
        WheelSpeedController(const WheelSpeedController&) = delete;
        WheelSpeedController& operator=(const WheelSpeedController&) = delete;

    private:
        static constexpr double WHEEL_RADIUS   = 0.057; //m
        static constexpr double WHEEL_DISTANCE = 0.128; //m
        static constexpr double VOLTAGE_MAX    = 12.0;  //Volt
        static constexpr double PI = 3.14159265359;
        static constexpr double RPM_MIN = 170;
        static constexpr double RPM_MAX = 350;
        static constexpr uint32_t PWM_DEFAULT_VALUE = 100000U;  // 10%
        static constexpr uint32_t PWM_MAX_VALUE     = 1000000U; // 1M, 60000~1000000, 1M
        static constexpr uint32_t PWM_FREQ = 10000000U;    //10M Hz

        int mPiHandle;
        int setLeftPwm(uint32_t aPwm);
        int setRightPwm(uint32_t aPwm);
        int setLeftDutyCycle(const double aDutyCycle);
        int setRightDutyCycle(const double aDutyCycle);
        uint32_t dutyCycleToPwm(const double aDutyCycle);
};
