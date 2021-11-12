#include <mobile_robotics_framework/BerryImu.h>

#include <string>
#include <system_error>

extern "C" {
#include <errno.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <linux/i2c-dev.h>
#include <i2c/smbus.h>
}

namespace {

namespace LIS3MDL {

constexpr auto ADDRESS     = 0x1C;

constexpr auto MAG_ID      = 0x3D;

constexpr auto WHO_AM_I    = 0x0F;

constexpr auto CTRL_REG1   = 0x20;
constexpr auto CTRL_REG2   = 0x21;
constexpr auto CTRL_REG3   = 0x22;
constexpr auto CTRL_REG4   = 0x23;
constexpr auto CTRL_REG5   = 0x24;

constexpr auto STATUS_REG  = 0x27;

constexpr auto OUT_X_L     = 0x28;
constexpr auto OUT_X_H     = 0x29;
constexpr auto OUT_Y_L     = 0x2A;
constexpr auto OUT_Y_H     = 0x2B;
constexpr auto OUT_Z_L     = 0x2C;
constexpr auto OUT_Z_H     = 0x2D;

constexpr auto TEMP_OUT_L  = 0x2E;
constexpr auto TEMP_OUT_H  = 0x2F;

constexpr auto INT_CFG     = 0x30;
constexpr auto INT_SRC     = 0x31;
constexpr auto INT_THS_L   = 0x32;
constexpr auto INT_THS_H   = 0x33;

} // namespace LIS3MDL

namespace LSM6DSL {

constexpr auto ADDRESS           = 0x6A;

constexpr auto GYR_ID            = 0x6A;

constexpr auto WHO_AM_I          = 0x0F;
constexpr auto RAM_ACCESS        = 0x01;
constexpr auto CTRL1_XL          = 0x10;
constexpr auto CTRL8_XL          = 0x17;
constexpr auto CTRL2_G           = 0x11;
constexpr auto CTRL10_C          = 0x19;
constexpr auto TAP_CFG1          = 0x58;
constexpr auto INT1_CTRL         = 0x0D;
constexpr auto CTRL3_C           = 0x12;
constexpr auto CTRL4_C           = 0x13;

constexpr auto STEP_COUNTER_L    = 0x4B;
constexpr auto STEP_COUNTER_H    = 0x4C;

constexpr auto OUTX_L_XL         = 0x28;
constexpr auto OUTX_H_XL         = 0x29;
constexpr auto OUTY_L_XL         = 0x2A;
constexpr auto OUTY_H_XL         = 0x2B;
constexpr auto OUTZ_L_XL         = 0x2C;
constexpr auto OUTZ_H_XL         = 0x2D;

constexpr auto OUT_L_TEMP        = 0x20;
constexpr auto OUT_H_TEMP        = 0x21;

constexpr auto OUTX_L_G          = 0x22;
constexpr auto OUTX_H_G          = 0x23;
constexpr auto OUTY_L_G          = 0x24;
constexpr auto OUTY_H_G          = 0x25;
constexpr auto OUTZ_L_G          = 0x26;
constexpr auto OUTZ_H_G          = 0x27;

constexpr auto TAP_CFG           = 0x58;
constexpr auto WAKE_UP_SRC       = 0x1B;
constexpr auto WAKE_UP_DUR       = 0x5C;
constexpr auto FREE_FALL         = 0x5D;
constexpr auto MD1_CFG           = 0x5E;
constexpr auto MD2_CFG           = 0x5F;
constexpr auto TAP_THS_6D        = 0x59;
constexpr auto INT_DUR2          = 0x5A;
constexpr auto WAKE_UP_THS       = 0x5B;
constexpr auto FUNC_SRC1         = 0x53;

} // namespace LSM6DSL 

namespace LSM9DS0 {

constexpr auto MAG_ADDRESS          = 0x1E;
constexpr auto ACC_ADDRESS          = 0x1E;
constexpr auto GYR_ADDRESS          = 0x6A;

constexpr auto ACC_ID               = 0xD4;
constexpr auto GYR_ID               = 0x49;

// Gyro Registers    
constexpr auto WHO_AM_I_G           = 0x0F;
constexpr auto CTRL_REG1_G          = 0x20;
constexpr auto CTRL_REG2_G          = 0x21;
constexpr auto CTRL_REG3_G          = 0x22;
constexpr auto CTRL_REG4_G          = 0x23;
constexpr auto CTRL_REG5_G          = 0x24;
constexpr auto REFERENCE_G          = 0x25;
constexpr auto STATUS_REG_G         = 0x27;
constexpr auto OUT_X_L_G            = 0x28;
constexpr auto OUT_X_H_G            = 0x29;
constexpr auto OUT_Y_L_G            = 0x2A;
constexpr auto OUT_Y_H_G            = 0x2B;
constexpr auto OUT_Z_L_G            = 0x2C;
constexpr auto OUT_Z_H_G            = 0x2D;
constexpr auto FIFO_CTRL_REG_G      = 0x2E;
constexpr auto FIFO_SRC_REG_G       = 0x2F;
constexpr auto INT1_CFG_G           = 0x30;
constexpr auto INT1_SRC_G           = 0x31;
constexpr auto INT1_THS_XH_G        = 0x32;
constexpr auto INT1_THS_XL_G        = 0x33;
constexpr auto INT1_THS_YH_G        = 0x34;
constexpr auto INT1_THS_YL_G        = 0x35;
constexpr auto INT1_THS_ZH_G        = 0x36;
constexpr auto INT1_THS_ZL_G        = 0x37;
constexpr auto INT1_DURATION_G      = 0x38;

// Accel and Magneto Registers 
constexpr auto OUT_TEMP_L_XM        = 0x05;
constexpr auto OUT_TEMP_H_XM        = 0x06;
constexpr auto STATUS_REG_M         = 0x07;
constexpr auto OUT_X_L_M            = 0x08;
constexpr auto OUT_X_H_M            = 0x09;
constexpr auto OUT_Y_L_M            = 0x0A;
constexpr auto OUT_Y_H_M            = 0x0B;
constexpr auto OUT_Z_L_M            = 0x0C;
constexpr auto OUT_Z_H_M            = 0x0D;
constexpr auto WHO_AM_I_XM          = 0x0F;
constexpr auto INT_CTRL_REG_M       = 0x12;
constexpr auto INT_SRC_REG_M        = 0x13;
constexpr auto INT_THS_L_M          = 0x14;
constexpr auto INT_THS_H_M          = 0x15;
constexpr auto OFFSET_X_L_M         = 0x16;
constexpr auto OFFSET_X_H_M         = 0x17;
constexpr auto OFFSET_Y_L_M         = 0x18;
constexpr auto OFFSET_Y_H_M         = 0x19;
constexpr auto OFFSET_Z_L_M         = 0x1A;
constexpr auto OFFSET_Z_H_M         = 0x1B;
constexpr auto REFERENCE_X          = 0x1C;
constexpr auto REFERENCE_Y          = 0x1D;
constexpr auto REFERENCE_Z          = 0x1E;
constexpr auto CTRL_REG0_XM         = 0x1F;
constexpr auto CTRL_REG1_XM         = 0x20;
constexpr auto CTRL_REG2_XM         = 0x21;
constexpr auto CTRL_REG3_XM         = 0x22;
constexpr auto CTRL_REG4_XM         = 0x23;
constexpr auto CTRL_REG5_XM         = 0x24;
constexpr auto CTRL_REG6_XM         = 0x25;
constexpr auto CTRL_REG7_XM         = 0x26;
constexpr auto STATUS_REG_A         = 0x27;
constexpr auto OUT_X_L_A            = 0x28;
constexpr auto OUT_X_H_A            = 0x29;
constexpr auto OUT_Y_L_A            = 0x2A;
constexpr auto OUT_Y_H_A            = 0x2B;
constexpr auto OUT_Z_L_A            = 0x2C;
constexpr auto OUT_Z_H_A            = 0x2D;
constexpr auto FIFO_CTRL_REG        = 0x2E;
constexpr auto FIFO_SRC_REG         = 0x2F;
constexpr auto INT_GEN_1_REG        = 0x30;
constexpr auto INT_GEN_1_SRC        = 0x31;
constexpr auto INT_GEN_1_THS        = 0x32;
constexpr auto INT_GEN_1_DURATION   = 0x33;
constexpr auto INT_GEN_2_REG        = 0x34;
constexpr auto INT_GEN_2_SRC        = 0x35;
constexpr auto INT_GEN_2_THS        = 0x36;
constexpr auto INT_GEN_2_DURATION   = 0x37;
constexpr auto CLICK_CFG            = 0x38;
constexpr auto CLICK_SRC            = 0x39;
constexpr auto CLICK_THS            = 0x3A;
constexpr auto TIME_LIMIT           = 0x3B;
constexpr auto TIME_LATENCY         = 0x3C;
constexpr auto TIME_WINDOW          = 0x3D;

} // namespace LSM9DS0

namespace LSM9DS1 {

constexpr auto MAG_ADDRESS      = 0x1C;  //Would be 0x1E if SDO_M is HIGH      
constexpr auto ACC_ADDRESS      = 0x6A;
constexpr auto GYR_ADDRESS      = 0x6A;  //Would be 0x6B if SDO_AG is HIGH

constexpr auto MAG_ID           = 0x3D;
constexpr auto GYR_ID           = 0x68;

// Accel/Gyro (XL/G) Registers
constexpr auto ACT_THS          = 0x04;
constexpr auto ACT_DUR          = 0x05;
constexpr auto INT_GEN_CFG_XL   = 0x06;
constexpr auto INT_GEN_THS_X_XL = 0x07;
constexpr auto INT_GEN_THS_Y_XL = 0x08;
constexpr auto INT_GEN_THS_Z_XL = 0x09;
constexpr auto INT_GEN_DUR_XL   = 0x0A;
constexpr auto REFERENCE_G      = 0x0B;
constexpr auto INT1_CTRL        = 0x0C;
constexpr auto INT2_CTRL        = 0x0D;
constexpr auto WHO_AM_I_XG      = 0x0F;
constexpr auto CTRL_REG1_G      = 0x10;
constexpr auto CTRL_REG2_G      = 0x11;
constexpr auto CTRL_REG3_G      = 0x12;
constexpr auto ORIENT_CFG_G     = 0x13;
constexpr auto INT_GEN_SRC_G    = 0x14;
constexpr auto OUT_TEMP_L       = 0x15;
constexpr auto OUT_TEMP_H       = 0x16;
constexpr auto STATUS_REG_0     = 0x17;
constexpr auto OUT_X_L_G        = 0x18;
constexpr auto OUT_X_H_G        = 0x19;
constexpr auto OUT_Y_L_G        = 0x1A;
constexpr auto OUT_Y_H_G        = 0x1B;
constexpr auto OUT_Z_L_G        = 0x1C;
constexpr auto OUT_Z_H_G        = 0x1D;
constexpr auto CTRL_REG4        = 0x1E;
constexpr auto CTRL_REG5_XL     = 0x1F;
constexpr auto CTRL_REG6_XL     = 0x20;
constexpr auto CTRL_REG7_XL     = 0x21;
constexpr auto CTRL_REG8        = 0x22;
constexpr auto CTRL_REG9        = 0x23;
constexpr auto CTRL_REG10       = 0x24;
constexpr auto INT_GEN_SRC_XL   = 0x26;
constexpr auto STATUS_REG_1     = 0x27;
constexpr auto OUT_X_L_XL       = 0x28;
constexpr auto OUT_X_H_XL       = 0x29;
constexpr auto OUT_Y_L_XL       = 0x2A;
constexpr auto OUT_Y_H_XL       = 0x2B;
constexpr auto OUT_Z_L_XL       = 0x2C;
constexpr auto OUT_Z_H_XL       = 0x2D;
constexpr auto FIFO_CTRL        = 0x2E;
constexpr auto FIFO_SRC         = 0x2F;
constexpr auto INT_GEN_CFG_G    = 0x30;
constexpr auto INT_GEN_THS_XH_G = 0x31;
constexpr auto INT_GEN_THS_XL_G = 0x32;
constexpr auto INT_GEN_THS_YH_G = 0x33;
constexpr auto INT_GEN_THS_YL_G = 0x34;
constexpr auto INT_GEN_THS_ZH_G = 0x35;
constexpr auto INT_GEN_THS_ZL_G = 0x36;
constexpr auto INT_GEN_DUR_G    = 0x37;

// Magneto Registers
constexpr auto OFFSET_X_REG_L_M = 0x05;
constexpr auto OFFSET_X_REG_H_M = 0x06;
constexpr auto OFFSET_Y_REG_L_M = 0x07;
constexpr auto OFFSET_Y_REG_H_M = 0x08;
constexpr auto OFFSET_Z_REG_L_M = 0x09;
constexpr auto OFFSET_Z_REG_H_M = 0x0A;
constexpr auto WHO_AM_I_M       = 0x0F;
constexpr auto CTRL_REG1_M      = 0x20;
constexpr auto CTRL_REG2_M      = 0x21;
constexpr auto CTRL_REG3_M      = 0x22;
constexpr auto CTRL_REG4_M      = 0x23;
constexpr auto CTRL_REG5_M      = 0x24;
constexpr auto STATUS_REG_M     = 0x27;
constexpr auto OUT_X_L_M        = 0x28;
constexpr auto OUT_X_H_M        = 0x29;
constexpr auto OUT_Y_L_M        = 0x2A;
constexpr auto OUT_Y_H_M        = 0x2B;
constexpr auto OUT_Z_L_M        = 0x2C;
constexpr auto OUT_Z_H_M        = 0x2D;
constexpr auto INT_CFG_M        = 0x30;
constexpr auto INT_SRC_M        = 0x30;
constexpr auto INT_THS_L_M      = 0x32;
constexpr auto INT_THS_H_M      = 0x33;

// WHO_AM_I Responses
constexpr auto WHO_AM_I_AG_RSP  = 0x68;
constexpr auto WHO_AM_I_M_RSP   = 0x3D;

} // namespace LSM9DS1

} // namespace

namespace mrf {

BerryImu::BerryImu(int device) {
    std::string filename("/dev/i2c-" + std::to_string(device));
    this->fd = open(filename.c_str(), O_RDWR);
    if (fd == -1) {
        throw std::system_error(errno, std::generic_category(),
                                "Failed to open " + filename);
    }

    // Detect Berry Imu version and enable imu
    try {
        // Version 1
        this->selectDevice(LSM9DS0::ACC_ADDRESS);
        auto accelId = this->readByte(LSM9DS0::WHO_AM_I_XM);

        this->selectDevice(LSM9DS0::GYR_ADDRESS);
        auto gyroId = this->readByte(LSM9DS0::WHO_AM_I_G);
        if (accelId == LSM9DS0::ACC_ID && gyroId == LSM9DS0::GYR_ID) {
            this->accelAddr = LSM9DS0::ACC_ADDRESS;
            this->gyroAddr = LSM9DS0::GYR_ADDRESS;
            this->magAddr = LSM9DS0::MAG_ADDRESS;

            // Enable gyro
            this->selectDevice(this->gyroAddr);
            this->writeByte(LSM9DS0::CTRL_REG1_G, 0b00001111); // Normal power mode, all axes enabled
            this->writeByte(LSM9DS0::CTRL_REG4_G, 0b00110000); // Continuos update, 2000 dps full scale

            // Enable accelerometer
            this->selectDevice(this->accelAddr);
            this->writeByte(LSM9DS0::CTRL_REG1_XM, 0b01100111); // xyz axis enabled, continuous update, 100 Hz
            this->writeByte(LSM9DS0::CTRL_REG2_XM, 0b00100000); // +/- 16G full scale

            // Enable magnetometer
            this->selectDevice(this->magAddr);
            this->writeByte(LSM9DS0::CTRL_REG5_XM, 0b11110000); // Temp enable, M data rate = 50Hz
            this->writeByte(LSM9DS0::CTRL_REG6_XM, 0b11110000); // +/-12gauss
            this->writeByte(LSM9DS0::CTRL_REG7_XM, 0b00000000); // Continuous-conversion mode
            
            return;
        }

        // Version 2
        this->selectDevice(LSM9DS1::MAG_ADDRESS);
        auto magId = this->readByte(LSM9DS1::WHO_AM_I_M);

        this->selectDevice(LSM9DS1::GYR_ADDRESS);
        gyroId = this->readByte(LSM9DS1::WHO_AM_I_XG);
        if (magId == LSM9DS1::MAG_ID && gyroId == LSM9DS1::GYR_ID) {
            this->accelAddr = LSM9DS1::ACC_ADDRESS;
            this->gyroAddr = LSM9DS1::GYR_ADDRESS;
            this->magAddr = LSM9DS1::MAG_ADDRESS;

            // Enable gyro
            this->selectDevice(this->gyroAddr);
            this->writeByte(LSM9DS1::CTRL_REG4, 0b00111000); // xyz axis enabled for gyro
            this->writeByte(LSM9DS1::CTRL_REG1_G, 0b10111000); // Gyro ODR = 476Hz, 2000 dps
            this->writeByte(LSM9DS1::ORIENT_CFG_G, 0b10111000); // Swap orientation

            // Enable accelerometer
            this->selectDevice(this->accelAddr);
            this->writeByte(LSM9DS1::CTRL_REG5_XL, 0b00111000); // xyz axis enabled
            this->writeByte(LSM9DS1::CTRL_REG6_XL, 0b00101000); // +/- 16g

            // Enable magnetometer
            this->selectDevice(this->magAddr);
            this->writeByte(LSM9DS1::CTRL_REG1_M, 0b10011100); // Temp compensation enabled,Low power mode mode,80Hz ODR
            this->writeByte(LSM9DS1::CTRL_REG2_M, 0b01000000); // +/-12gauss
            this->writeByte(LSM9DS1::CTRL_REG3_M, 0b00000000); // Continuous update
            this->writeByte(LSM9DS1::CTRL_REG4_M, 0b00000000); // Lower power mode for z axis

            return;
        }

        // Version 3
        this->selectDevice(LIS3MDL::ADDRESS);
        magId = this->readByte(LIS3MDL::WHO_AM_I);

        this->selectDevice(LSM6DSL::ADDRESS);
        gyroId = this->readByte(LSM6DSL::WHO_AM_I);
        if (magId == LIS3MDL::MAG_ID && gyroId == LSM6DSL::GYR_ID) {
            this->accelAddr = LSM6DSL::ADDRESS;
            this->gyroAddr = LSM6DSL::ADDRESS;
            this->magAddr = LIS3MDL::ADDRESS;

            // Enable gyro
            this->selectDevice(this->gyroAddr);
            this->writeByte(LSM6DSL::CTRL2_G, 0b10011100); // ODR 3.3 kHz, 2000 dps

            // Enable accelerometer
            this->selectDevice(this->accelAddr);
            this->writeByte(LSM6DSL::CTRL1_XL, 0b10011111); // ODR 3.33 kHz, +/- 8g, BW = 400hz
            this->writeByte(LSM6DSL::CTRL8_XL, 0b11001000); // Low pass filter enabled, BW9, composite filter
            this->writeByte(LSM6DSL::CTRL3_C, 0b01000100); // Enable Block Data update, increment during multi byte read

            // Enable magnetometer
            this->selectDevice(this->magAddr);
            this->writeByte(LIS3MDL::CTRL_REG1, 0b11011100); // Temp sesnor enabled, High performance, ODR 80 Hz, FAST ODR disabled and Selft test disabled
            this->writeByte(LIS3MDL::CTRL_REG2, 0b00100000); // +/- 8 gauss
            this->writeByte(LIS3MDL::CTRL_REG3, 0b00000000); // Continuous-conversion mode

            return;
        }

        throw std::system_error(std::make_error_code(std::errc::not_supported),
                                "Failed to detect Berry Imu version");
    } catch (...) {
        close(this->fd);
        throw;
    }
}

BerryImu::~BerryImu() {
    close(this->fd);
}

void BerryImu::selectDevice(int addr) {
    if (ioctl(this->fd, I2C_SLAVE, addr) == -1) {
        throw std::system_error(errno, std::generic_category(), 
                                "Failed to select imu i2c device");
    }
}

uint8_t BerryImu::readByte(uint8_t cmd) {
    auto result = i2c_smbus_read_byte_data(this->fd, cmd);
    if (fd == -1) {
        throw std::system_error(errno, std::generic_category(), 
                                "Failed to read byte from imu");
    }
    return result;
}

void BerryImu::readBlock(uint8_t cmd, uint8_t size, uint8_t data[]) {
    auto bytesRead = i2c_smbus_read_i2c_block_data(this->fd, cmd, size, data);
    if (bytesRead != size) {
        throw std::system_error(errno, std::generic_category(), 
                                "Failed to read " + std::to_string(size) + " bytes from imu");
    }
}

void BerryImu::writeByte(uint8_t reg, uint8_t value) {
    if (i2c_smbus_write_byte_data(this->fd, reg, value) == -1) {
        throw std::system_error(errno, std::generic_category(), 
                                "Failed to write byte to imu");
    }
}

} // namespace mrf
