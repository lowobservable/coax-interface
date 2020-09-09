#pragma once

#define RX_ERROR_LOSS_OF_MID_BIT_TRANSITION -1
#define RX_ERROR_PARITY -2
#define RX_ERROR_INVALID_END_SEQUENCE -4
#define RX_ERROR_OVERFLOW -8

class SPICoaxTransceiver
{
public:
    SPICoaxTransceiver(int chipSelectPin);

    bool begin();

    int receive(uint16_t *buffer, size_t bufferSize);

private:
    int _chipSelectPin;
};
