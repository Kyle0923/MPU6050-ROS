#include <ros/ros.h>
#include <cmath>
#include <stdexcept>
#include <pigpiod_if2.h>
#include "mpu6050_gpio_if.hpp"

static constexpr unsigned MPU6050_I2C_ADDRESS = 0x68;

static constexpr unsigned PWR_MGMT_1   = 0x6B;
static constexpr unsigned SMPLRT_DIV   = 0x19;
static constexpr unsigned CONFIG       = 0x1A;
static constexpr unsigned GYRO_CONFIG  = 0x1B;
static constexpr unsigned ACCEL_CONFIG = 0x1C;
static constexpr unsigned INT_ENABLE   = 0x38;
static constexpr unsigned ACCEL_XOUT_H = 0x3B;
static constexpr unsigned ACCEL_YOUT_H = 0x3D;
static constexpr unsigned ACCEL_ZOUT_H = 0x3F;
static constexpr unsigned GYRO_XOUT_H  = 0x43;
static constexpr unsigned GYRO_YOUT_H  = 0x45;
static constexpr unsigned GYRO_ZOUT_H  = 0x47;

/*
GYRO FS_SEL Full Scale Range LSB Sensitivity
    0[0x00] | ± 250 °/s  | 131  LSB/°/s
    1[0x08] | ± 500 °/s  | 65.5 LSB/°/s
    2[0x10] | ± 1000 °/s | 32.8 LSB/°/s
    3[0x18] | ± 2000 °/s | 16.4 LSB/°/s
ACCEL AFS_SEL Full Scale Range LSB Sensitivity
    0[0x00] | ±2g  | 16384 LSB/g
    1[0x08] | ±4g  | 8192  LSB/g
    2[0x10] | ±8g  | 4096  LSB/g
    3[0x18] | ±16g | 2048  LSB/g
 */

static constexpr unsigned ACCEL_AFS_SEL = 0x00;
static constexpr unsigned GYRO_FS_SEL   = 0x00;

static constexpr double ACCEL_SENSITIVITY = 16384.0;
static constexpr double GYRO_SENSITIVITY  = 131.0;

static constexpr double PI = 3.14159265359;
static constexpr double g = 9.81; // m/s^2

int16_t Mpu6050GpioIF::readRawData(const unsigned addr)
{
    uint16_t highByte, lowByte;
    highByte = static_cast<uint8_t>(i2c_read_word_data(mPiHandle, mI2CHandle, addr));
    lowByte = static_cast<uint8_t>(i2c_read_word_data(mPiHandle, mI2CHandle, addr + 1U));
    uint16_t unsignValue = (highByte << 8) | lowByte;
    return static_cast<int16_t>(unsignValue);
}

void Mpu6050GpioIF::calibration(const float duration, const int samples)
{
    Mpu6050Data total {0,0,0,0,0,0};
    Mpu6050Data newData {0,0,0,0,0,0};
    for(int counter = 0; counter < samples; ++counter)
    {
        newData = readData();
        total.accel_x += newData.accel_x;
        total.accel_y += newData.accel_y;
        total.accel_z += newData.accel_z - g;
        total.gyro_x += newData.gyro_x;
        total.gyro_y += newData.gyro_y;
        total.gyro_z += newData.gyro_z;
        ros::Duration(duration / samples).sleep();
    }
    mAccelXBias = total.accel_x / samples;
    mAccelYBias = total.accel_y / samples;
    mAccelZBias = total.accel_z / samples;
    mGyroXBias = total.gyro_x / samples;
    mGyroYBias = total.gyro_y / samples;
    mGyroZBias = total.gyro_z / samples;
    // ROS_INFO("Ax: %f", mAccelXBias);
    // ROS_INFO("Ay: %f", mAccelYBias);
    // ROS_INFO("Az: %f", mAccelZBias);
    // ROS_INFO("Gx: %f", mGyroXBias);
    // ROS_INFO("Gy: %f", mGyroYBias);
    // ROS_INFO("Gz: %f", mGyroZBias);

}

Mpu6050Data Mpu6050GpioIF::readData()
{
    Mpu6050Data Mpu6050Data{0, 0, 0, 0, 0, 0};

    int16_t rawAx = readRawData(ACCEL_XOUT_H);
    int16_t rawAy = readRawData(ACCEL_YOUT_H);
    int16_t rawAz = readRawData(ACCEL_ZOUT_H);
    int16_t rawGx = readRawData(GYRO_XOUT_H);
    int16_t rawGy = readRawData(GYRO_YOUT_H);
    int16_t rawGz = readRawData(GYRO_ZOUT_H);

    // unit: m/s^2
    Mpu6050Data.accel_x = static_cast<double>(rawAx) * g / ACCEL_SENSITIVITY - mAccelXBias;
    Mpu6050Data.accel_y = static_cast<double>(rawAy) * g / ACCEL_SENSITIVITY - mAccelYBias;
    Mpu6050Data.accel_z = static_cast<double>(rawAz) * g / ACCEL_SENSITIVITY - mAccelZBias;

    // unit: rad/s
    Mpu6050Data.gyro_x = static_cast<double>(rawGx) * PI / 180 / GYRO_SENSITIVITY - mGyroXBias;
    Mpu6050Data.gyro_y = static_cast<double>(rawGy) * PI / 180 / GYRO_SENSITIVITY - mGyroYBias;
    Mpu6050Data.gyro_z = static_cast<double>(rawGz) * PI / 180 / GYRO_SENSITIVITY - mGyroZBias;

    // ROS_INFO("Ax: %f", Mpu6050Data.accel_x);
    // ROS_INFO("Ay: %f", Mpu6050Data.accel_y);
    // ROS_INFO("Az: %f", Mpu6050Data.accel_z);
    // ROS_INFO("Gx: %f", Mpu6050Data.gyro_x);
    // ROS_INFO("Gy: %f", Mpu6050Data.gyro_y);
    // ROS_INFO("Gz: %f", Mpu6050Data.gyro_z);

    return Mpu6050Data;
}

void Mpu6050GpioIF::initMPU6050()
{
    int status = 0;
    status |= i2c_write_byte_data(mPiHandle, mI2CHandle, SMPLRT_DIV, 0x07); // 1kHz
    status |= i2c_write_byte_data(mPiHandle, mI2CHandle, PWR_MGMT_1, 0x01); // clock: x-gyro
    status |= i2c_write_byte_data(mPiHandle, mI2CHandle, CONFIG, 0);        // mpu6050 freq = 8 kHz
    status |= i2c_write_byte_data(mPiHandle, mI2CHandle, GYRO_CONFIG, GYRO_FS_SEL); // gryo range
    status |= i2c_write_byte_data(mPiHandle, mI2CHandle, ACCEL_CONFIG, ACCEL_AFS_SEL); // accel range
    status |= i2c_write_byte_data(mPiHandle, mI2CHandle, INT_ENABLE, 0x01);     // interrupt enable, DATA_RDY_EN
    if (status != 0)
    {
        throw std::runtime_error("init MPU6050 failed");
    }
}

Mpu6050GpioIF::Mpu6050GpioIF():
    mAccelXBias(0), mAccelYBias(0), mAccelZBias(0), mGyroXBias(0), mGyroYBias(0), mGyroZBias(0)
{
    mPiHandle = pigpio_start(nullptr, nullptr);
    if (mPiHandle < 0)
    {
        throw std::runtime_error("unable to start pigpio");
    }
    mI2CHandle = i2c_open(mPiHandle, 1U, MPU6050_I2C_ADDRESS, 0);
    if (mI2CHandle < 0)
    {
        throw std::runtime_error("unable to open I2C");
    }
    initMPU6050();
}

Mpu6050GpioIF::~Mpu6050GpioIF()
{
    i2c_close(mPiHandle, mI2CHandle);
    pigpio_stop(mPiHandle);
}