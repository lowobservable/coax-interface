#pragma once

#define COAX_REGISTER_STATUS 0x1
#define COAX_REGISTER_STATUS_RX_ACTIVE 0x20

class SPICoaxTransceiver
{
public:
    SPICoaxTransceiver(const int csPin);

    bool begin();

    uint8_t readRegister(const int index);
    int receive(uint16_t *buffer, const size_t bufferSize);

private:
    int _csPin;
};
