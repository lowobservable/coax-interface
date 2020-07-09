#pragma once

#define ERROR_RX_RECEIVER_ACTIVE -5
#define ERROR_RX_TIMEOUT -2
#define ERROR_RX_OVERFLOW -3

class NewCoaxDataBus;

enum NewCoaxReceiverState { Disabled, Idle, Receiving, Received };

class NewCoaxReceiver
{
public:
    NewCoaxReceiver(NewCoaxDataBus &dataBus) : _dataBus(dataBus) { };

    void begin();
    void enable();
    void disable();
    int receive(uint16_t *buffer, size_t bufferSize, uint16_t timeout);
    void activeInterrupt();
    void dataAvailableInterrupt();
    void errorInterrupt();

private:
    NewCoaxDataBus &_dataBus;
    volatile NewCoaxReceiverState _state = Disabled;
    volatile uint16_t _error;
    volatile uint16_t *_buffer;
    volatile size_t _bufferSize;
    volatile int _bufferCount;

    void reset();
    uint16_t read();
};

class NewCoaxDataBus
{
public:
    void setMode(int mode);
    uint16_t read();
    void write(uint16_t word);
    uint16_t encode(uint16_t word);
    uint16_t decode(uint16_t word);
};