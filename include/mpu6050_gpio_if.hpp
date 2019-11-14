#ifndef MPU6050_GPIO_IF_HPP
#define MPU6050_GPIO_IF_HPP

struct Mpu6050Data
{
    double accel_x;
    double accel_y;
    double accel_z;
    double gyro_x;
    double gyro_y;
    double gyro_z;
};

class Mpu6050GpioIF
{
    public:
        Mpu6050GpioIF();
        ~Mpu6050GpioIF();
        Mpu6050Data readData();
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

#endif // MPU6050_GPIO_IF_HPP