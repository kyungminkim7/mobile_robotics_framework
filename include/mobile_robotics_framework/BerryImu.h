#pragma once

#include <cstdint>

namespace mrf {

class BerryImu {
public:
    explicit BerryImu(int device);
    ~BerryImu();

private:
    void selectDevice(int addr);
    uint8_t readByte();
    void readBlock(uint8_t cmd, uint8_t size, uint8_t data[]);

private:
    int fd;
};

} // mrf
