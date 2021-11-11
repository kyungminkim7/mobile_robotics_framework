#pragma once

#include <cstdint>

namespace mrf {

class BerryImu {
public:
    explicit BerryImu(int device);
    ~BerryImu();

private:
    void selectDevice(int addr);

    uint8_t readByte(uint8_t cmd);
    void readBlock(uint8_t cmd, uint8_t size, uint8_t data[]);

    void writeByte(uint8_t reg, uint8_t value);

private:
    int fd;
    int accelAddr;
    int gyroAddr;
    int magAddr;
};

} // mrf
