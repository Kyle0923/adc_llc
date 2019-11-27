#ifndef MPU6050_IF_HPP
#define MPU6050_IF_HPP

struct Mpu6050Data
{
    double accel_x;
    double accel_y;
    double accel_z;
    double gyro_x;
    double gyro_y;
    double gyro_z;
};

class Mpu6050IF
{
    public:
        Mpu6050IF(const int piHandle);
        Mpu6050IF() = delete;
        ~Mpu6050IF();
        Mpu6050Data readData();
        double getTheta();
        void updateTheta(const uint32_t tick);
        void disable();

    private:
        bool mEnable;
        int mPiHandle;
        int mI2CHandle;
        uint32_t mLastTick;
        double mTheta;  //angle w.r.t X-asix
        double mAccelXBias;
        double mAccelYBias;
        double mAccelZBias;
        double mGyroXBias;
        double mGyroYBias;
        double mGyroZBias;
        void initMPU6050();
        void calibration(const float duration, const int samples);
        int16_t readRawData(const unsigned addr);
};

#endif // MPU6050_IF_HPP