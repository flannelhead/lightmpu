/*
    A lightweight Arduino library for reading and processing motion information
    from a MPU-6050 sensor.
    Copyright (C) 2016 Sakari Kapanen

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

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
#define MPU_CLK_PLL_ZGYRO 3

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
    int16_t gThresh;
    int16_t g2;
    int16_t gyroDivider;
    int32_t alpha;
    int32_t alphaComplement;
};

const mpuconfig MPU_DEFAULT_CONFIG = {
    .disableTemp = true,
    .lowpass = 3,
    .sampleRateDivider = 4,
    .gyroRange = 3,
    .accelRange = 0,
    .enableInterrupt = true
};

int mpuWriteRegister(const uint8_t addr, const uint8_t reg, const uint8_t value,
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

void mpuApplyOffsets(int16_t * const data, const int16_t * const offsets) {
    for (uint8_t i = 0; i < 7; i++) {
        data[i] += offsets[i];
    }
}

int mpuSetup(const uint8_t addr, const mpuconfig * const config) {
    int status;
    // Wake up and disable temperature measurement if asked for
    status = mpuWriteRegister(addr, MPU_PWR_MGMT_1,
        MPU_CLK_PLL_ZGYRO | (config->disableTemp ? MPU_TEMP_DIS : 0), false);
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

// Pitch calculation by means of the complementary filter

const int16_t MPU_GYRO_RANGE[] = { 250, 500, 1000, 2000 };
const int8_t MPU_ACCEL_RANGE[] = { 2, 4, 8, 16 };
const int16_t ANGLE_SCALE_FACTOR = 256;
void mpuSetupFilter(const mpuconfig * const config, mpufilter * const filter,
    const int16_t alpha = 16) {
    filter->alpha = alpha;
    filter->alphaComplement = 512 - filter->alpha;

    int16_t g = (INT16_MAX / MPU_ACCEL_RANGE[config->accelRange]) >> 8;
    filter->g2 = g*g;
    filter->gThresh = g*g * 3 / 2;

    int32_t c = (int32_t)(1 + config->sampleRateDivider) * g * 314 *
                MPU_GYRO_RANGE[config->gyroRange] / INT16_MAX,
            d = 18000UL * 1000 / ANGLE_SCALE_FACTOR;
    filter->gyroDivider = d / c;
}

void mpuUpdatePitch(mpufilter * const filter, int16_t * const data,
    int16_t * const pitch) {
    // Use only the 8 most significant bits of the accelerometer readings to
    // take advantage from single-instruction multiplication (muls).
    // The readings won't have more than 8 bits of real information anyway.
    int8_t *data8 = (int8_t*)data;
    int8_t ax = data8[(MPU_ACC_X << 1) + 1],
           ay = data8[(MPU_ACC_Y << 1) + 1],
           az = data8[(MPU_ACC_Z << 1) + 1];
    int16_t gy = data[MPU_GYRO_Y];

    int16_t ayz2 = (int16_t)(ay*ay) + (int16_t)(az*az),
            ax2 = ax*ax, a2 = ayz2 + ax2;

    int16_t gyroTerm = *pitch + gy / filter->gyroDivider;
    int16_t accTerm = 0;
    if (a2 < filter->gThresh) {
        /* accTerm = (int32_t)ANGLE_SCALE_FACTOR * ax * (filter->g2 + ax2/6) / */
        /*     filter->g2; */
        accTerm = ANGLE_SCALE_FACTOR * ax;
        *pitch = (filter->alphaComplement * gyroTerm -
            filter->alpha * accTerm) / 512;
    } else {
        *pitch = gyroTerm;
    }
}

#endif

