// An example of using the lightmpu library

#include <Wire.h>
#include "lightmpu.h"

const int MPU_ADDR = 0x68;
int16_t mpuData[7];
volatile bool gMpuInterrupt = false;

const mpuconfig mpuConfig = MPU_DEFAULT_CONFIG;
mpufilter mpuFilter;
float pitch = 0.0;

void mpuInterrupt() {
    gMpuInterrupt = true;
}

void setup() {
    Serial.begin(57600);
    Wire.begin();
    mpuSetup(MPU_ADDR, &mpuConfig);
    mpuSetupFilter(&mpuConfig, &mpuFilter, 0.05);
    attachInterrupt(digitalPinToInterrupt(2), mpuInterrupt, RISING);
    mpuReadIntStatus(MPU_ADDR);
}

void loop() {
    if (gMpuInterrupt) {
        gMpuInterrupt = false;
        mpuReadIntStatus(MPU_ADDR);
        mpuReadRawData(MPU_ADDR, mpuData);
        mpuUpdatePitch(&mpuFilter, mpuData, &pitch);
        Serial.println(pitch);
    }
}

