#pragma once

#include <cstdint>

#include <eigen3/Eigen/Core>

namespace mrf {

class BerryImu {
public:
    explicit BerryImu(int device);
    ~BerryImu();

    Eigen::Vector3f getLinearAcceleration();
    Eigen::Vector3f getAngularVelocity();

private:
    void selectDevice(int addr);

    uint8_t readByte(uint8_t cmd);
    void readBlock(uint8_t cmd, uint8_t size, uint8_t data[]);

    void writeByte(uint8_t reg, uint8_t value);

private:
    int fd; // Imu i2c file descriptor

    // I2c device addresses
    int accelAddr;
    int gyroAddr;
    int magAddr;

    // I2c cmds
    int accelReadCmd;
    int gyroReadCmd;
    int magReadCmd;

    float accelGain_mps2;
    float gyroGain_rad;
};

} // mrf
