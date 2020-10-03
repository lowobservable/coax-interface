#pragma once

#define COAX_REGISTER_STATUS 0x1
#define COAX_REGISTER_STATUS_RX_ACTIVE 0x20

#define COAX_REGISTER_DEVICE_ID 0xf

class SPICoaxTransceiver
{
public:
    SPICoaxTransceiver(const int csPin);

    bool begin();

    uint8_t readRegister(const uint8_t index);

    int transmit(const uint16_t *buffer, const size_t bufferCount);
    int receive(uint16_t *buffer, const size_t bufferSize);

    inline bool isActive()
    {
        return readRegister(COAX_REGISTER_STATUS) & COAX_REGISTER_STATUS_RX_ACTIVE;
    };

private:
    int _csPin;

    bool spiSetup();
    bool spiTransfer(const uint8_t *transmitBuffer, uint8_t *receiveBuffer, const size_t count);
};