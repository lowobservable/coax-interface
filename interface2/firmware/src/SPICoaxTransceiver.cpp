#include <Arduino.h>
#include <SPI.h>

#include "SPICoaxTransceiver.h"

#define COMMAND_READ_REGISTER 0x2
#define COMMAND_RX 0x5

SPISettings spiSettings(4000000, MSBFIRST, SPI_MODE0);

SPICoaxTransceiver::SPICoaxTransceiver(int chipSelectPin) :
    _chipSelectPin(chipSelectPin)
{
}

bool SPICoaxTransceiver::begin()
{
    SPI.begin();

    pinMode(_chipSelectPin, OUTPUT);
    digitalWrite(_chipSelectPin, HIGH);

    return true;
}

uint8_t SPICoaxTransceiver::readRegister(uint8_t index)
{
    SPI.beginTransaction(spiSettings);

    digitalWrite(_chipSelectPin, LOW);

    SPI.transfer(COMMAND_READ_REGISTER | (index << 4));

    uint8_t value = SPI.transfer(0);

    digitalWrite(_chipSelectPin, HIGH);

    SPI.endTransaction();

    return value;
}

int SPICoaxTransceiver::receive(uint16_t *buffer, size_t bufferSize)
{
    SPI.beginTransaction(spiSettings);

    digitalWrite(_chipSelectPin, LOW);

    SPI.transfer(COMMAND_RX);

    int index = 0;
    uint16_t error = 0;

    do {
        uint8_t msb = SPI.transfer(0);
        uint8_t lsb = SPI.transfer(0);

        uint16_t value = (msb << 8) | lsb;

        if (value & 0x8000) {
            error = value & 0x3ff;
            break;
        } else if (value & 0x4000) {
            break;
        }

        buffer[index] = value & 0x3ff;

        index++;
    } while(index < bufferSize);

    digitalWrite(_chipSelectPin, HIGH);

    SPI.endTransaction();

    if (error != 0) {
        return (-1) * error;
    }

    return index;
}
