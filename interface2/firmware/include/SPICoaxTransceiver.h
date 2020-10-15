#pragma once

#define COAX_REGISTER_STATUS 0x1
#define COAX_REGISTER_STATUS_RX_ERROR 0x40
#define COAX_REGISTER_STATUS_RX_ACTIVE 0x20
#define COAX_REGISTER_STATUS_TX_COMPLETE 0x08
#define COAX_REGISTER_STATUS_TX_ACTIVE 0x04

#define COAX_REGISTER_CONTROL 0x2
#define COAX_REGISTER_CONTROL_LOOPBACK 0x01
#define COAX_REGISTER_CONTROL_TX_PARITY (1 << 3)
#define COAX_REGISTER_CONTROL_RX_PARITY (1 << 6)

#define COAX_REGISTER_DEVICE_ID 0xf

enum class CoaxParity
{
    Odd = 0,
    Even = 1
};

class SPICoaxTransceiver
{
public:
    SPICoaxTransceiver(const int csPin);

    bool begin();

    void reset();

    uint8_t readRegister(const uint8_t index);
    void writeRegister(const uint8_t index, const uint8_t value, const uint8_t mask);

    int transmit(const uint16_t *buffer, const size_t bufferCount);
    int receive(uint16_t *buffer, const size_t bufferSize);

    void setLoopback(const bool loopback);
    void setTXParity(const CoaxParity parity);
    void setRXParity(const CoaxParity parity);

    inline bool isTXComplete()
    {
        return readRegister(COAX_REGISTER_STATUS) & COAX_REGISTER_STATUS_TX_COMPLETE;
    }

    inline bool isRXActive()
    {
        return readRegister(COAX_REGISTER_STATUS) & COAX_REGISTER_STATUS_RX_ACTIVE;
    };

private:
    int _csPin;

    bool spiSetup();
    bool spiTransfer(const uint8_t *transmitBuffer, uint8_t *receiveBuffer, const size_t count);
};
