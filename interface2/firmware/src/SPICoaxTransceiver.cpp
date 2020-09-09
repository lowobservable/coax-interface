#include <Arduino.h>
#include <SPI.h>

#include "SPICoaxTransceiver.h"

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

        // vvv
        Serial.print(msb, BIN);
        Serial.print(" _ ");
        Serial.println(lsb, BIN);
        // ^^^

        if (msb & 0x80) {
            error = ((msb << 8) | lsb) & 0x3ff;
            break;
        } else if (msb & 0x40) {
            break;
        }

        uint16_t word = ((msb << 8) | lsb) & 0x3ff;

        buffer[index] = word;

        index++;
    } while(index < bufferSize);

    digitalWrite(_chipSelectPin, HIGH);

    SPI.endTransaction();

    if (error != 0) {
        return (-1) * error;
    }

    return index;
}
