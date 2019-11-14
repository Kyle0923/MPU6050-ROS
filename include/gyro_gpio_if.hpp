#ifndef GYRO_GPIO_IF_HPP
#define GYRO_GPIO_IF_HPP

struct GyroData
{
    double accel_x;
    double accel_y;
    double accel_z;
    double gyro_x;
    double gyro_y;
    double gyro_z;
};

class GyroGpioIF
{
    public:
        GyroGpioIF();
        ~GyroGpioIF();
        GyroData readGyroData();
        void calibration(const float duration, const int samples);

    private:
        int mPiHandle;
        unsigned mI2CHandle;
        void initMPU6050();
        int16_t readRawData(const unsigned addr);
        double mAccelXBias;
        double mAccelYBias;
        double mAccelZBias;
        double mGyroXBias;
        double mGyroYBias;
        double mGyroZBias;
};

#endif // GYRO_GPIO_IF_HPP