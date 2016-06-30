#ifndef I2CMASTER_H
#define I2CMASTER_H

void i2cInit(int bitrate) {
    PRR &= ~_BV(PRTWI);
    TWSR &= ~(_BV(TWPS0) | _BV(TWPS1));
    TWBR = (F_CPU / bitrate - 16) / 2;
}

#endif

