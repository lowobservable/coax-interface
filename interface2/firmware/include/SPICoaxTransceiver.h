#pragma once

#define RX_ERROR_LOSS_OF_MID_BIT_TRANSITION -1
#define RX_ERROR_PARITY -2
#define RX_ERROR_INVALID_END_SEQUENCE -4
#define RX_ERROR_OVERFLOW -8

#define COAX_REGISTER_STATUS 0x1
#define COAX_REGISTER_STATUS_RX_ACTIVE 0x20

class SPICoaxTransceiver
{
public:
    SPICoaxTransceiver(int chipSelectPin);

    bool begin();

    uint8_t readRegister(uint8_t index);

    int receive(uint16_t *buffer, size_t bufferSize);

private:
    int _chipSelectPin;
};
