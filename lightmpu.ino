// An example of using the lightmpu library

#include <Wire.h>
#include "lightmpu.h"

// 0 = plotting
// 1 = benchmark
// 2 = calibration
#define MODE 0

#define BENCHMARK_SAMPLES 1000
#define CALIBRATION_SAMPLES 5000

const int MPU_ADDR = 0x68;
int16_t mpuData[7];
const int16_t mpuOffsets[] = { -345, 211, 432, 0, 33, 6, 18 };
int32_t sums[7] = { 0 };
volatile bool gMpuInterrupt = false;

const mpuconfig mpuConfig = MPU_DEFAULT_CONFIG;
mpufilter mpuFilter;
float pitch = 0.0;
int nFrames = 0;
unsigned long lastTime = 0;

void mpuInterrupt() {
    gMpuInterrupt = true;
}

void setup() {
    Serial.begin(57600);
    Wire.begin();
    Wire.setClock(400000L);
    mpuSetup(MPU_ADDR, &mpuConfig);
    mpuSetupFilter(&mpuConfig, &mpuFilter);
    attachInterrupt(digitalPinToInterrupt(2), mpuInterrupt, RISING);
    mpuReadIntStatus(MPU_ADDR);
}

void loop() {
    if (gMpuInterrupt) {
        gMpuInterrupt = false;
        mpuReadIntStatus(MPU_ADDR);
        mpuReadRawData(MPU_ADDR, mpuData);
        #if MODE != 2
            mpuApplyOffsets(mpuData, mpuOffsets);
        #endif
        mpuUpdatePitch(&mpuFilter, mpuData, &pitch);
        #if MODE == 0
            Serial.println(pitch);
        #elif MODE == 1
            nFrames++;
            if (nFrames == BENCHMARK_SAMPLES) {
                nFrames = 0;
                int curTime = millis();
                Serial.println(curTime - lastTime);
                lastTime = curTime;
            }
        #elif MODE == 2
            nFrames++;
            for (uint8_t i = 0; i < 7; i++) {
                sums[i] += mpuData[i];
            }
            if (nFrames == CALIBRATION_SAMPLES) {
                Serial.println("Sensor offsets:");
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

