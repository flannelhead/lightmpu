// Implements configuring and reading a MPU-6050 motion sensor as a lightweight
// C library for Arduino. Also contains a complementary filter for obtaining
// pitch and roll angles from the raw data.
//
// TODO: compare this with the DMP results

#ifndef LIGHTMPU_H
#define LIGHTMPU_H

#include <Wire.h>

// Register addresses and bits as per the MPU-6050 datasheet
// http://43zrtwysvxb2gf29r5o0athu.wpengine.netdna-cdn.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf

#define MPU_PWR_MGMT_1 0x6B
#define MPU_TEMP_DIS bit(3)

#define MPU_CONFIG 0x1A
#define MPU_SMPRT_DIV 0x19

#define MPU_GYRO_CONFIG 0x1B
#define MPU_ACCEL_CONFIG 0x1C

#define MPU_INT_ENABLE 0x38
#define MPU_DATA_RDY_EN bit(0)
#define MPU_MOT_EN bit(6)

#define MPU_INT_STATUS 0x3A
#define MPU_DATA_RDY_INT bit(0)

#define MPU_ACCEL_XOUT_H 0x3B
#define MPU_ACCEL_XOUT_L 0x3C
#define MPU_ACCEL_YOUT_H 0x3D
#define MPU_ACCEL_YOUT_L 0x3E
#define MPU_ACCEL_ZOUT_H 0x3F
#define MPU_ACCEL_ZOUT_L 0x40

#define MPU_TEMP_OUT_H 0x41
#define MPU_TEMP_OUT_L 0x42

#define MPU_GYRO_XOUT_H 0x43
#define MPU_GYRO_XOUT_L 0x44
#define MPU_GYRO_YOUT_H 0x45
#define MPU_GYRO_YOUT_L 0x46
#define MPU_GYRO_ZOUT_H 0x47
#define MPU_GYRO_ZOUT_L 0x48

#define MPU_ACC_X 0
#define MPU_ACC_Y 1
#define MPU_ACC_Z 2
#define MPU_TEMP 3
#define MPU_GYRO_X 4
#define MPU_GYRO_Y 5
#define MPU_GYRO_Z 6

struct mpuconfig {
    bool disableTemp;
    // Lowpass filter bandwidth
    // 0 = 260 Hz
    // 1 = 184 Hz
    // 2 = 94 Hz
    // 3 = 44 Hz
    // 4 = 21 Hz
    // 5 = 10 Hz
    // 6 = 5 Hz
    uint8_t lowpass;
    uint8_t sampleRateDivider;
    // Gyro full scale range
    // 0 = +- 250 deg/s
    // 1 = +- 500 deg/s
    // 2 = +- 1000 deg/s
    // 3 = +- 2000 deg/s
    uint8_t gyroRange;
    // Accelerometer full scale range
    // 0 = +- 2 g
    // 1 = +- 4 g
    // 2 = +- 8 g
    // 3 = +- 16 g
    uint8_t accelRange;
    bool enableInterrupt;
};

struct mpufilter {
    float dt;
    uint16_t gyroSensitivity;
    float gyroFactor;
    float filterParam;
};

const mpuconfig MPU_DEFAULT_CONFIG = {
    .disableTemp = true,
    .lowpass = 6,
    .sampleRateDivider = 4,
    .gyroRange = 3,
    .accelRange = 3,
    .enableInterrupt = true
};

struct mpudata {
    int16_t accX, accY, accZ, temp,
        gyroX, gyroY, gyroZ;
};

int mpuWriteRegister(const int addr, const uint8_t reg, const uint8_t value,
    const bool stop) {
    Wire.beginTransmission(addr);
    if (Wire.write(reg) != 1) return -1;
    if (Wire.write(value) != 1) return -2;
    return Wire.endTransmission(stop);
}

int mpuReadRegisters(const uint8_t addr, const uint8_t firstReg,
    const uint8_t len, uint8_t * const data) {
    int status;
    Wire.beginTransmission(addr);
    if (Wire.write(firstReg) != 1) return -1;
    status = Wire.endTransmission(false);
    if (status != 0) return status;
    if (Wire.requestFrom(addr, len, (uint8_t)true) != len) return -2;
    for (uint8_t i = 0; i < len; i++) data[i] = Wire.read();
    return 0;
}

int mpuSetup(const int addr, const mpuconfig * const config) {
    int status;
    // Wake up and disable temperature measurement if asked for
    status = mpuWriteRegister(addr, MPU_PWR_MGMT_1,
        config->disableTemp ? MPU_TEMP_DIS : 0, false);
    if (status != 0) return status;
    // Configure the low pass filter
    if (config->lowpass > 6) return -10;
    status = mpuWriteRegister(addr, MPU_CONFIG, config->lowpass, false);
    if (status != 0) return status;
    // Configure gyro and accelerometer sensitivities
    if (config->gyroRange > 3) return -11;
    status = mpuWriteRegister(addr, MPU_GYRO_CONFIG,
        config->gyroRange << 3, false);
    if (status != 0) return status;
    if (config->accelRange > 3) return -12;
    status = mpuWriteRegister(addr, MPU_ACCEL_CONFIG,
        config->accelRange << 3, false);
    if (status != 0) return status;
    // Configure the sample rate
    status = mpuWriteRegister(addr, MPU_SMPRT_DIV, config->sampleRateDivider,
        false);
    if (status != 0) return status;
    // Configure the interrupt
    status = mpuWriteRegister(addr, MPU_INT_ENABLE,
        config->enableInterrupt ? MPU_DATA_RDY_EN : 0, true);
    return status;
}

// Pitch and roll calculation by means of the complementary filter
// NOTE: while this code doesn't depend on the DMP or the FIFO buffer,
// you should be aware that the processor does have to keep up with the
// sample rate in order for the calculations to be accurate.

const int MPU_GYRO_RANGE[] = { 250, 500, 1000, 2000 };
void mpuSetupFilter(const mpuconfig * const config, mpufilter * const filter,
    const float filterParam = 0.08) {
    filter->dt = (float)(1 + config->sampleRateDivider) /
        (config->lowpass != 0 ? 1000 : 8000);
    filter->gyroSensitivity =
        32768 / (MPU_GYRO_RANGE[config->gyroRange] / 180 * PI);
    filter->gyroFactor = filter->dt / filter->gyroSensitivity;
    filter->filterParam = filterParam;
}

void mpuUpdatePitch(mpufilter * const filter, int16_t * const data,
    float * const pitch) {
    int32_t sqr = sqrt((int32_t)data[MPU_ACC_Y] * data[MPU_ACC_Y] +
        (int32_t)data[MPU_ACC_Z] * data[MPU_ACC_Z]);
    if (sqr == 0) return;
    float accPitch = atan2(data[MPU_ACC_X], sqr);
    *pitch = (1 - filter->filterParam) *
        (*pitch + filter->gyroFactor * data[MPU_GYRO_Y])
        + filter->filterParam * accPitch;
}

void mpuUpdateRoll(mpufilter * const filter, int16_t * const data,
    float * const roll) {
    int32_t sqr = sqrt((int32_t)data[MPU_ACC_X] * data[MPU_ACC_X] +
        (int32_t)data[MPU_ACC_Z] * data[MPU_ACC_Z]);
    if (sqr == 0) return;
    float accPitch = atan2(data[MPU_ACC_Y], sqr);
    *roll = (1 - filter->filterParam) *
        (*roll + filter->gyroFactor * data[MPU_GYRO_X])
        + filter->filterParam * accPitch;
}

int mpuReadIntStatus(const uint8_t addr) {
    uint8_t tmp;
    mpuReadRegisters(addr, MPU_INT_STATUS, 1, &tmp);
    return tmp;
}

int mpuReadRawData(const uint8_t addr, int16_t * const data) {
    int status;
    Wire.beginTransmission(addr);
    if (Wire.write(MPU_ACCEL_XOUT_H) != 1) return -1;
    status = Wire.endTransmission(false);
    if (status != 0) return status;
    if (Wire.requestFrom(addr, (uint8_t)14, (uint8_t)true) != 14) return -2;
    for (uint8_t i = 0; i < 7; i++) {
        data[i] = Wire.read() << 8 | Wire.read();
    }
    return 0;
}

#endif

