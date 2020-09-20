#pragma once

#define COAX_REGISTER_STATUS 0x1

class SPICoaxTransceiver
{
public:
    SPICoaxTransceiver(const int csPin);

    bool begin();

    uint8_t readRegister(const int index);

private:
    int _csPin;
};
