#include <Arduino.h>

#include <Indicators.h>
#include <ICE40.h>
#include <SPICoaxTransceiver.h>

#include <pins.h>

const uint8_t ice40Bitstream[] = {
#include "bitstream.inc"
};

Indicators indicators(LED_STATUS_PIN, LED_TX_PIN, LED_RX_PIN, LED_ERROR_PIN);

ICE40 ice40(ICE40_CS_PIN, ICE40_SCK_PIN, ICE40_SDI_PIN, ICE40_CRESET_PIN,
        ICE40_CDONE_PIN);

SPICoaxTransceiver coax(ICE40_CS_PIN);

volatile enum {
    COAX_INTERRUPT_STATE_IDLE,
    COAX_INTERRUPT_STATE_RECEIVING,
    COAX_INTERRUPT_STATE_RECEIVED,
    COAX_INTERRUPT_STATE_ERROR
} coaxInterruptState = COAX_INTERRUPT_STATE_IDLE;

#define COAX_BUFFER_SIZE 1 + (3696 * 2)

uint16_t coaxInterruptBuffer[COAX_BUFFER_SIZE];
volatile size_t coaxInterruptBufferCount = 0;
volatile int coaxInterruptError = 0;

void handleCoaxInterrupt()
{
    if (coaxInterruptState != COAX_INTERRUPT_STATE_IDLE) {
        // ...
        return;
    }

    coaxInterruptState = COAX_INTERRUPT_STATE_RECEIVING;

    int error = 0;

    uint16_t *buffer = coaxInterruptBuffer;
    size_t bufferSize = COAX_BUFFER_SIZE;
    int bufferCount = 0;

    bool isActive;
    int count;

    do {
        // Determine if the receiver is active before reading from the FIFO to
        // avoid a race condition where the FIFO is empty, the receiver is
        // active but is inactive by the time we check the status.
        isActive = coax.isActive();

        count = coax.receive(buffer, bufferSize);

        if (count < 0) {
            error = count;
            break;
        }

        bufferCount += count;

        buffer += count;
        bufferSize -= count;
    } while (isActive || count == (int) bufferSize);

    if (error != 0) {
        coaxInterruptBufferCount = 0;
        coaxInterruptError = error;

        coaxInterruptState = COAX_INTERRUPT_STATE_ERROR;
    } else {
        coaxInterruptBufferCount = bufferCount;
        coaxInterruptError = 0;

        coaxInterruptState = COAX_INTERRUPT_STATE_RECEIVED;
    }
}

void setup()
{
    indicators.begin();

    SerialUSB.begin();

    delay(500);

    indicators.setStatus(CONFIGURING);

    while (!ice40.isConfigured()) {
        if (!ice40.configure(ice40Bitstream, sizeof(ice40Bitstream))) {
            indicators.error();

            delay(1000);
        }
    }

    while (!coax.begin()) {
        indicators.error();

        delay(1000);
    }

    pinMode(COAX_IRQ_PIN, INPUT);

    attachInterrupt(digitalPinToInterrupt(COAX_IRQ_PIN), handleCoaxInterrupt, RISING);

    delay(500);

    indicators.setStatus(RUNNING);
}

void handleReceiveData(const uint16_t *buffer, const size_t count)
{
    indicators.rx();

    SerialUSB.print("RX ");
    SerialUSB.print(count);
    SerialUSB.println(" word(s)");
}

void handleReceiveError(const int error)
{
    indicators.error();

    SerialUSB.print("RX ");

    if (error == -1) {
        SerialUSB.println("loss of mid-bit transition error");
    } else if (error == -2) {
        SerialUSB.println("parity error");
    } else if (error == -4) {
        SerialUSB.println("invalid end sequence error");
    } else if (error == -8) {
        SerialUSB.println("coax_buffered_rx overflow error");
    } else {
        SerialUSB.print("unknown ");
        SerialUSB.print(error);
        SerialUSB.println("error");
    }
}

void loop()
{
    static uint16_t coaxBuffer[COAX_BUFFER_SIZE];

    if (coaxInterruptState == COAX_INTERRUPT_STATE_RECEIVED) {
        size_t count = coaxInterruptBufferCount;

        memcpy(coaxBuffer, coaxInterruptBuffer, count);

        coaxInterruptState = COAX_INTERRUPT_STATE_IDLE;

        handleReceiveData(coaxBuffer, count);
    }

    if (coaxInterruptState == COAX_INTERRUPT_STATE_ERROR) {
        int error = coaxInterruptError;

        coaxInterruptState = COAX_INTERRUPT_STATE_IDLE;

        handleReceiveError(error);
    }

    if (SerialUSB.available()) {
        char input = SerialUSB.read();

        // ...

        SerialUSB.flush();
    }
}
