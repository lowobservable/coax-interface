#include <Arduino.h>
#include <SPI.h>

#include <ICE40.h>

ICE40::ICE40(const int csPin, const int sckPin, const int sdiPin, const int cresetPin,
        const int cdonePin) :
    _csPin(csPin),
    _sckPin(sckPin),
    _sdiPin(sdiPin),
    _cresetPin(cresetPin),
    _cdonePin(cdonePin),
    _isConfigured(false)
{
}

bool ICE40::configure(const uint8_t *bitstream, const size_t bitstreamCount)
{
    _isConfigured = false;

    SPI.end();

    pinMode(_csPin, OUTPUT);
    pinMode(_sckPin, OUTPUT);
    pinMode(_sdiPin, OUTPUT);
    pinMode(_cresetPin, OUTPUT);
    pinMode(_cdonePin, INPUT);

    digitalWrite(_cresetPin, LOW);

    digitalWrite(_sckPin, HIGH);
    digitalWrite(_csPin, LOW);

    delayMicroseconds(100);

    digitalWrite(_cresetPin, HIGH);

    delayMicroseconds(1000);

    for (int index = 0; index < 8; index++) {
        clock();
    }

    for (int index = 0; index < bitstreamCount; index++) {
        uint8_t byte = bitstream[index];

        for (int bitIndex = 7; bitIndex >= 0; bitIndex--) {
            digitalWrite(_sdiPin, (byte >> bitIndex) & 0x01);
            clock();
        }
    }

    bool done = digitalRead(_cdonePin);

    if (done) {
        for (int index = 0; index < 49; index++) {
            clock();
        }
    }

    _isConfigured = done;

    return _isConfigured;
}
