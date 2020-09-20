#pragma once

class ICE40
{
public:
    ICE40(const int csPin, const int sckPin, const int sdiPin, const int cresetPin,
            const int cdonePin);

    bool configure(const uint8_t *bitstream, const size_t bitstreamCount);

    inline bool isConfigured()
    {
        return _isConfigured;
    }

private:
    int _csPin;
    int _sckPin;
    int _sdiPin;
    int _cresetPin;
    int _cdonePin;
    bool _isConfigured;

    inline void clock()
    {
        digitalWrite(_sckPin, LOW);
        digitalWrite(_sckPin, HIGH);
    }
};
