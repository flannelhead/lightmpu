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

// An example of using the lightmpu library

#include <Wire.h>
#include "lightmpu.h"

// 0 = plotting
// 1 = benchmark
// 2 = calibration
#define MODE 0

#define BENCHMARK_SAMPLES 1000
#define CALIBRATION_SAMPLES 5000
#define ALPHA 0

const int MPU_ADDR = 0x68;
int16_t mpuData[7];
const int16_t mpuOffsets[] = { 423, -276, 2190, 0, 55, 6, 15 };
int32_t sums[7] = { 0 };
volatile bool gMpuInterrupt = false;

mpuconfig mpuConfig = MPU_DEFAULT_CONFIG;
mpufilter mpuFilter;
int16_t pitch = 0;
int nFrames = 0;
unsigned long lastTime = 0;

void mpuInterrupt() {
    gMpuInterrupt = true;
}

void setup() {
    mpuConfig.sampleRateDivider = 0;
    Serial.begin(115200);
    Wire.begin();
    Wire.setClock(400000L);
    digitalWrite(SDA, LOW);
    digitalWrite(SCL, LOW);
    mpuSetup(MPU_ADDR, &mpuConfig);
    mpuSetupFilter(&mpuConfig, &mpuFilter, ALPHA);
    attachInterrupt(digitalPinToInterrupt(2), mpuInterrupt, RISING);
    mpuReadIntStatus(MPU_ADDR);
    #if MODE == 1
        Serial.println(F("Benchmarking, please stand by..."));
    #elif MODE == 2
        Serial.println(F("Calibrating, please stand by..."));
    #endif
}

void loop() {
    if (gMpuInterrupt) {
        gMpuInterrupt = false;
        mpuReadIntStatus(MPU_ADDR);
        mpuReadRawData(MPU_ADDR, mpuData);
        #if MODE != 2
            mpuApplyOffsets(mpuData, mpuOffsets);
            mpuUpdatePitch(&mpuFilter, mpuData, &pitch);
        #endif
        #if MODE == 0
            Serial.println(pitch);
        #elif MODE == 1
            nFrames++;
            if (nFrames == BENCHMARK_SAMPLES) {
                nFrames = 0;
                unsigned long curTime = millis();
                Serial.println(curTime - lastTime);
                lastTime = curTime;
            }
        #elif MODE == 2
            nFrames++;
            for (uint8_t i = 0; i < 7; i++) {
                sums[i] += mpuData[i];
            }
            if (nFrames == CALIBRATION_SAMPLES) {
                Serial.println(F("Sensor offsets:"));
                int val;
                for (uint8_t i = 0; i < 7; i++) {
                    val = -sums[i] / CALIBRATION_SAMPLES;
                    if (i == 2) val += INT16_MAX /
                        pow(2, mpuConfig.accelRange + 1);
                    Serial.print(val);
                    Serial.print(" ");
                }
                Serial.println("");
            }
        #endif
    }
}

