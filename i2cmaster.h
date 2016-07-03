#ifndef I2CMASTER_H
#define I2CMASTER_H

#include <util/twi.h>

void i2cInit(int32_t bitrate) {
    PRR &= ~_BV(PRTWI);
    TWSR &= ~(_BV(TWPS0) | _BV(TWPS1));
    TWBR = (F_CPU / bitrate - 16) / 2;
}

inline bool i2cWaitForStatus(uint8_t status) {
    while (!(TWCR & _BV(TWINT)));
    return TWSR == status;
}

bool i2cSendStart() {
    TWCR = _BV(TWINT) | _BV(TWSTA) | _BV(TWEN);
    while (!(TWCR & _BV(TWINT)));
    return TWSR == TW_START || TWSR == TW_REP_START;
}

void i2cSendStop() {
    TWCR = _BV(TWINT) | _BV(TWSTO) | _BV(TWEN);
}

bool i2cSendSLA(uint8_t addr, uint8_t mode, uint8_t status) {
    TWDR = (addr << 1) | mode;
    TWCR = _BV(TWINT) | _BV(TWEN);
    return i2cWaitForStatus(status);
}

bool i2cTransmitTo(uint8_t addr) {
    return i2cSendSLA(addr, TW_WRITE, TW_MT_SLA_ACK);
}

bool i2cReceiveFrom(uint8_t addr) {
    return i2cSendSLA(addr, TW_READ, TW_MR_SLA_ACK);
}

bool i2cWriteBytes(uint8_t * const bytes, int nBytes) {
    bool ret = true;
    for (int i = 0; i < nBytes; i++) {
        TWDR = bytes[i];
        TWCR = _BV(TWINT) | _BV(TWEN);
        if (!i2cWaitForStatus(TW_MT_DATA_ACK)) {
            ret = false;
            break;
        }
    }
    return ret;
}

bool i2cReadNextByte(uint8_t * const byte, const bool ack) {
    uint8_t twcr = _BV(TWINT) | _BV(TWEN);
    if (ack) twcr |= _BV(TWEA);
    TWCR = twcr;
    if (i2cWaitForStatus(ack ? TW_MR_DATA_ACK : TW_MR_DATA_NACK)) {
        *byte = TWDR;
        return true;
    }
    return false;
}

bool i2cReadBytes(uint8_t * const bytes, int8_t nBytes) {
    for (int8_t i = 0; i < nBytes; i++) {
        if (!i2cReadNextByte(bytes + i, i != nBytes - 1)) return false;
    }
    return true;
}

#endif

