#include <Arduino.h>
#include <SPI.h>

#include <SPICoaxTransceiver.h>

#define COAX_COMMAND_READ_REGISTER 0x2
#define COAX_COMMAND_RX 0x5

SPISettings spiSettings(8000000, MSBFIRST, SPI_MODE0);

SPICoaxTransceiver::SPICoaxTransceiver(const int csPin) : _csPin(csPin)
{
}

bool SPICoaxTransceiver::begin()
{
    pinMode(_csPin, OUTPUT);
    digitalWrite(_csPin, HIGH);

    SPI.begin();

    return true;
}

uint8_t SPICoaxTransceiver::readRegister(const int index)
{
    SPI.beginTransaction(spiSettings);

    digitalWrite(_csPin, LOW);

    SPI.transfer(COAX_COMMAND_READ_REGISTER | (index << 4));

    uint8_t value = SPI.transfer(0);

    digitalWrite(_csPin, HIGH);

    SPI.endTransaction();

    return value;
}

int SPICoaxTransceiver::receive(uint16_t *buffer, const size_t bufferSize)
{
    SPI.beginTransaction(spiSettings);

    digitalWrite(_csPin, LOW);

    SPI.transfer(COAX_COMMAND_RX);

    size_t count = 0;
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

        buffer[count] = value & 0x3ff;

        count++;
    } while (count < bufferSize);

    digitalWrite(_csPin, HIGH);

    SPI.endTransaction();

    if (error != 0) {
        return (-1) * error;
    }

    return count;
}
