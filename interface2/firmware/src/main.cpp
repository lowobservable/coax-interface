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

SPICoaxTransceiver coax(ICE40_CS_PIN, COAX_RESET_PIN);

volatile enum {
    COAX_INTERRUPT_STATE_IDLE,
    COAX_INTERRUPT_STATE_DISABLED,
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
        isActive = coax.isRXActive();

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

void attachCoaxInterrupt()
{
    pinMode(COAX_IRQ_PIN, INPUT);

    attachInterrupt(digitalPinToInterrupt(COAX_IRQ_PIN), handleCoaxInterrupt, RISING);
}

void detachCoaxInterrupt()
{
    detachInterrupt(digitalPinToInterrupt(COAX_IRQ_PIN));
}

int testLoopbackTransmitReceive(const size_t count, const CoaxParity txParity,
        const CoaxParity rxParity, const int expectedTransmitResult,
        const int expectedReceiveResult)
{
    // Disable the interrupt during the self test as the receive interrupt can
    // interrupt the transmit routine and cause the SPI transaction to be
    // corrupted.
    coaxInterruptState = COAX_INTERRUPT_STATE_DISABLED;

    coax.setLoopback(true);

    coax.setTXParity(txParity);
    coax.setRXParity(rxParity);

    coax.reset();

    uint16_t transmitBuffer[256];
    uint16_t receiveBuffer[256];

    for (size_t index = 0; index < 256; index++) {
        transmitBuffer[index] = index;
    }

    int result = 0;
    int transmitResult;
    int receiveResult;

    transmitResult = coax.transmit(transmitBuffer, count);

    if (transmitResult != expectedTransmitResult) {
        result = -1;
        goto CLEANUP;
    }

    receiveResult = coax.receive(receiveBuffer, count);

    if (receiveResult != expectedReceiveResult) {
        result = -2;
        goto CLEANUP;
    }

    if (expectedReceiveResult == count) {
        if (transmitResult != count) {
            result = -3;
            goto CLEANUP;
        }

        if (receiveResult != count) {
            result = -4;
            goto CLEANUP;
        }

        for (size_t index = 0; index < count; index++) {
            if (receiveBuffer[index] != transmitBuffer[index]) {
                result = -5;
                goto CLEANUP;
            }
        }
    }

CLEANUP:
    coax.reset();

    coaxInterruptState = COAX_INTERRUPT_STATE_IDLE;

    coax.setLoopback(false);

    return result;
}

void selfTest()
{
    SERIAL_PORT_MONITOR.println("SELF TEST");

    bool allPassed = true;

    SERIAL_PORT_MONITOR.print("  - Parity mismatch: ");

    int result = testLoopbackTransmitReceive(1, CoaxParity::Odd, CoaxParity::Even, 1, -2);

    if (result == 0) {
        SERIAL_PORT_MONITOR.println("PASS");
    } else {
        allPassed = false;

        SERIAL_PORT_MONITOR.print("FAIL, result = ");
        SERIAL_PORT_MONITOR.println(result);
    }

    for (size_t count = 1; count <= 64; count++) {
        SERIAL_PORT_MONITOR.print("  - ");
        SERIAL_PORT_MONITOR.print(count);
        SERIAL_PORT_MONITOR.print(" words: ");

        result = testLoopbackTransmitReceive(count, CoaxParity::Even, CoaxParity::Even, count, count);

        if (result == 0) {
            SERIAL_PORT_MONITOR.println("PASS");
        } else {
            allPassed = false;

            SERIAL_PORT_MONITOR.print("FAIL, result = ");
            SERIAL_PORT_MONITOR.println(result);
        }
    }

    if (allPassed) {
        SERIAL_PORT_MONITOR.println("done, all PASS");
    } else {
        SERIAL_PORT_MONITOR.println("done, some FAILs");
    }
}

void handleReceiveData(const uint16_t *buffer, const size_t count)
{
    indicators.rx();

    SERIAL_PORT_MONITOR.print("RX ");
    SERIAL_PORT_MONITOR.print(count);
    SERIAL_PORT_MONITOR.println(" word(s)");
}

void handleReceiveError(const int error)
{
    indicators.error();

    SERIAL_PORT_MONITOR.print("RX ");

    if (error == -1) {
        SERIAL_PORT_MONITOR.println("loss of mid-bit transition error");
    } else if (error == -2) {
        SERIAL_PORT_MONITOR.println("parity error");
    } else if (error == -4) {
        SERIAL_PORT_MONITOR.println("invalid end sequence error");
    } else if (error == -8) {
        SERIAL_PORT_MONITOR.println("coax_buffered_rx overflow error");
    } else {
        SERIAL_PORT_MONITOR.print("unknown ");
        SERIAL_PORT_MONITOR.print(error);
        SERIAL_PORT_MONITOR.println("error");
    }
}

void testReadRegister()
{
    SERIAL_PORT_MONITOR.println();

    SERIAL_PORT_MONITOR.println("REGISTERS");

    uint8_t status = coax.readRegister(COAX_REGISTER_STATUS);

    SERIAL_PORT_MONITOR.print("  Status = ");
    SERIAL_PORT_MONITOR.println(status);

    SERIAL_PORT_MONITOR.print("    RX Error    = ");
    SERIAL_PORT_MONITOR.println(status & COAX_REGISTER_STATUS_RX_ERROR ? "Y" : "N");

    SERIAL_PORT_MONITOR.print("    RX Active   = ");
    SERIAL_PORT_MONITOR.println(status & COAX_REGISTER_STATUS_RX_ACTIVE ? "Y" : "N");

    SERIAL_PORT_MONITOR.print("    TX Complete = ");
    SERIAL_PORT_MONITOR.println(status & COAX_REGISTER_STATUS_TX_COMPLETE ? "Y" : "N");

    SERIAL_PORT_MONITOR.print("    TX Active   = ");
    SERIAL_PORT_MONITOR.println(status & COAX_REGISTER_STATUS_TX_ACTIVE ? "Y" : "N");

    uint8_t control = coax.readRegister(COAX_REGISTER_CONTROL);

    SERIAL_PORT_MONITOR.print("  Control = ");
    SERIAL_PORT_MONITOR.println(control);

    SERIAL_PORT_MONITOR.print("    Loopback    = ");
    SERIAL_PORT_MONITOR.println(control & COAX_REGISTER_CONTROL_LOOPBACK ? "Y" : "N");

    SERIAL_PORT_MONITOR.print("    TX Parity   = ");
    SERIAL_PORT_MONITOR.println(control & COAX_REGISTER_CONTROL_TX_PARITY ? "Even" : "Odd");

    SERIAL_PORT_MONITOR.print("    RX Parity   = ");
    SERIAL_PORT_MONITOR.println(control & COAX_REGISTER_CONTROL_RX_PARITY ? "Even" : "Odd");
}

void setup()
{
    indicators.begin();

    SERIAL_PORT_MONITOR.begin(115200, SERIAL_8N1);

    SERIAL_PORT_MONITOR.println("COAX");

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

    attachCoaxInterrupt();

    delay(500);

    indicators.setStatus(RUNNING);
}

void loop()
{
    static uint16_t buffer[COAX_BUFFER_SIZE];

    if (coaxInterruptState == COAX_INTERRUPT_STATE_RECEIVED) {
        size_t count = coaxInterruptBufferCount;

        memcpy(buffer, coaxInterruptBuffer, count);

        coaxInterruptState = COAX_INTERRUPT_STATE_IDLE;

        handleReceiveData(buffer, count);
    }

    if (coaxInterruptState == COAX_INTERRUPT_STATE_ERROR) {
        int error = coaxInterruptError;

        coaxInterruptState = COAX_INTERRUPT_STATE_IDLE;

        handleReceiveError(error);
    }

    if (SerialUSB.available()) {
        // ...
    }

    if (SERIAL_PORT_MONITOR.available()) {
        char input = SERIAL_PORT_MONITOR.read();

        if (input == 'R') {
            SERIAL_PORT_MONITOR.println("RESET");
            coax.reset();
        } else if (input == 'r') {
            testReadRegister();
        } else if (input == 'T') {
            selfTest();
        }
    }
}
