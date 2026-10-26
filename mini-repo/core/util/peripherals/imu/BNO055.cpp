#include "BNO055.h"
#include <cstdint>

// Register addresses
#define PWR_MODE 0x3E
#define OPR_MODE 0x3D
#define SYS_TRIGGER 0x3F
#define BNO055_ACCEL_DATA_X_LSB 0x08
#define BNO055_GYR_DATA_X_LSB 0x14
#define BNO055_EUL_HEADING_LSB 0x1A

/*
We added these two functions since they're definitely beyond what we expect from you as recruits
I'd be impressed if you understood the math behind them.
Basically just know that quats are 4 axis (1 real, 3 imaginary) numbers that allow us 
to nicely talk about rotation, and the functions spit out pitch roll and yaw. 
*/

BNO055::BNO055(I2C &i2c, uint8_t addr) noexcept
    : _i2c_p(&i2c),
      _i2c(i2c),
      _res(NC),
      chip_addr(addr),
      cantReadDataCount(0),
      imuAngles{}
{
}

BNO055::BNO055(I2C &i2c, uint8_t addr, PinName p_reset) noexcept
    : _i2c_p(&i2c),
      _i2c(i2c),
      _res(p_reset, 1),
      chip_addr(addr),
      cantReadDataCount(0),
      imuAngles{}
{
}

void BNO055::init() noexcept{
    reset();
    dt[0] = PWR_MODE;
    dt[1] = 0x00;
    _i2c.write(chip_addr, dt, 2, false);

    dt[0] = OPR_MODE;
    dt[1] = 0x0C;
    _i2c.write(chip_addr, dt, 2, false);

    std::this_thread::sleep_for(std::chrono::milliseconds(20)); // 7 ms is minimum time
}

void BNO055::reset() noexcept{
    // hardware reset if pin is connected
    if (_res.is_connected()) {
        _res.write(v:0);
        std::this_thread::sleep_for(std::chrono::milliseconds(1)); // 20ns is minimum time
        _res.write(v:1);
    }   
    // software reset if pin is not connected
    else {
        dt[0] = SYS_TRIGGER;
        dt[1] = 0x20; 
        _i2c.write(chip_addr, dt, 2, false);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(700)); // 650ms is minimum time
}

void BNO055::get_accel(BNO055_VECTOR_TypeDef *la){
    if (cantReadDataCount > 0 && cantReadDataCount < 50) {
        cantReadDataCount++;
        return;
    } else if (cantReadDataCount >= 50) {
        cantReadDataCount = 1;
    }
    int16_t x,y,z;

    dt[0] = BNO055_ACCEL_DATA_X_LSB;

    int writeResult = _i2c.write(chip_addr, dt, 1, true);  // returns 0 if successful, 1 if not
    // runs if write was successful
    if (!writeResult)  {
        if (cantReadDataCount > 0) {
            printf("RESET IMU\n");
            reset();
            cantReadDataCount = 0;
        }
        _i2c.read(chip_addr, dt, 6, false);
        x = dt[1] << 8 | dt[0]; // makes 16 bit number
        y = dt[3] << 8 | dt[2];
        z = dt[5] << 8 | dt[4];

        la->x = (double)x / 100.0f;
        la->y = (double)y / 100.0f;
        la->z = (double)z / 100.0f;
    } else {
        cantReadDataCount++;
    }
}

void BNO055::get_gyro(BNO055_VECTOR_TypeDef *gr) {
    if (cantReadDataCount > 0 && cantReadDataCount < 50) {
        cantReadDataCount++;
        return;
    } else if (cantReadDataCount >= 50) {
        cantReadDataCount = 1;
    }
    int16_t x,y,z;

    dt[0] = BNO055_GYR_DATA_X_LSB;

    int writeResult = _i2c.write(chip_addr, dt, 1, true);
    if (!writeResult) {
        if (cantReadDataCount > 0) {
            printf("RESET IMU\n");
            reset();
            cantReadDataCount = 0;
        }
        _i2c.read(chip_addr, dt, 6, false);
        x = dt[1] << 8 | dt[0];
        y = dt[3] << 8 | dt[2];
        z = dt[5] << 8 | dt[4];

        gr->x = (double)x / 16.0;
        gr->y = (double)y / 16.0;
        gr->z = (double)z / 16.0;
    }
    else {
        cantReadDataCount++;
    }
}

void BNO055::change_fusion_mode(uint8_t mode) {
    dt[0] = OPR_MODE;
    dt[1] = mode;

    _i2c.write(chip_addr, dt, 2, false);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));

}

IMU::EulerAngles BNO055::read() {

    dt[0] = 0x1A;

    _i2c.write(chip_addr, dt, 1, true);
    _i2c.read(chip_addr, dt, 6, false);

    int16_t yaw = dt[1] << 8 | dt[0];
    int16_t roll = dt[3] << 8 | dt[2];
    int16_t pitch = dt[5] << 8 | dt[4];

    imuAngles.yaw = yaw/16.0;
    imuAngles.roll = roll/16.0;
    imuAngles.pitch = pitch/16.0;

    return imuAngles;

}

IMU::EulerAngles BNO055::getImuAngles() {

    if (cantReadDataCount > 0 && cantReadDataCount < 50) {
        cantReadDataCount++;
        return imuAngles;
    } else if (cantReadDataCount >= 50) {
        cantReadDataCount = 1;
    }
    int16_t x,y,z;

    dt[0] = BNO055_EUL_HEADING_LSB;

    int writeResult = _i2c.write(chip_addr, dt, 1, true);
    if (!writeResult) {
        if (cantReadDataCount > 0) {
            printf("RESET IMU\n");
            reset();
            cantReadDataCount = 0;
        }
        _i2c.read(chip_addr, dt, 6, false);
        x = dt[1] << 8 | dt[0];
        y = dt[3] << 8 | dt[2];
        z = dt[5] << 8 | dt[4];

        imuAngles.yaw = (double)x / 16.0;
        imuAngles.roll = (double)y / 16.0;
        imuAngles.pitch = (double)z / 16.0;
    }
    else {
        cantReadDataCount++;
    }
    return imuAngles;
}

void BNO055::get_quaternion(BNO055_QUATERNION_TypeDef *result)
{
    if (cantReadDataCount > 0 && cantReadDataCount < 50) {
        cantReadDataCount++;
        return;
    } else if (cantReadDataCount >= 50) {
        cantReadDataCount = 1;
    }
    int16_t w,x,y,z;

    dt[0] = BNO055_QUATERNION_W_LSB;
    int writeResult = _i2c.write(chip_addr, dt, 1, true);
    if (!writeResult)  {
        if (cantReadDataCount > 0) {
            printf("RESET IMU\n");
            reset();
            cantReadDataCount = 0;
        }
        _i2c.read(chip_addr, dt, 8, false);
        w = dt[1] << 8 | dt[0];
        x = dt[3] << 8 | dt[2];
        y = dt[5] << 8 | dt[4];
        z = dt[7] << 8 | dt[6];

        result->w = (double)w / 16384.0f;
        result->x = (double)x / 16384.0f;
        result->y = (double)y / 16384.0f;
        result->z = (double)z / 16384.0f;
    } else {
        cantReadDataCount++;
    }
}

void BNO055::get_angular_position_quat(IMU::EulerAngles *result){

    BNO055_QUATERNION_TypeDef q;
    get_quaternion(&q);

    float roll  = atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y)) * 180 / PI;
    float pitch = asin(2 * q.w * q.y - q.x * q.z) * 180 / PI;
    float yaw   = atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)) * 180 / PI;

    memcpy(&result->roll, &roll, sizeof(float));
    memcpy(&result->pitch, &pitch, sizeof(float));
    memcpy(&result->yaw, &yaw, sizeof(float));
}

