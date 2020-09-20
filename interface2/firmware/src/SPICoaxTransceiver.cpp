#include <Arduino.h>
#include <SPI.h>

#include <SPICoaxTransceiver.h>

#define COAX_COMMAND_READ_REGISTER 0x2

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
