#include <Arduino.h>

#include <Indicators.h>
#include <ICE40.h>
#include <SPICoaxTransceiver.h>

#include <pins.h>

#define SERIAL_MONITOR Serial1
#define SERIAL_INTERFACE SerialUSB

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
    SERIAL_MONITOR.println("SELF TEST");

    bool allPassed = true;

    SERIAL_MONITOR.print("  - Parity mismatch: ");

    int result = testLoopbackTransmitReceive(1, CoaxParity::Odd, CoaxParity::Even, 1, -2);

    if (result == 0) {
        SERIAL_MONITOR.println("PASS");
    } else {
        allPassed = false;

        SERIAL_MONITOR.print("FAIL, result = ");
        SERIAL_MONITOR.println(result);
    }

    for (size_t count = 1; count <= 64; count++) {
        SERIAL_MONITOR.print("  - ");
        SERIAL_MONITOR.print(count);
        SERIAL_MONITOR.print(" words: ");

        result = testLoopbackTransmitReceive(count, CoaxParity::Even, CoaxParity::Even, count, count);

        if (result == 0) {
            SERIAL_MONITOR.println("PASS");
        } else {
            allPassed = false;

            SERIAL_MONITOR.print("FAIL, result = ");
            SERIAL_MONITOR.println(result);
        }
    }

    if (allPassed) {
        SERIAL_MONITOR.println("done, all PASS");
    } else {
        SERIAL_MONITOR.println("done, some FAILs");
    }
}

void handleReceiveData(const uint16_t *buffer, const size_t count)
{
    indicators.rx();

    SERIAL_MONITOR.print("RX ");
    SERIAL_MONITOR.print(count);
    SERIAL_MONITOR.println(" word(s)");
}

void handleReceiveError(const int error)
{
    indicators.error();

    SERIAL_MONITOR.print("RX ");

    if (error == -1) {
        SERIAL_MONITOR.println("loss of mid-bit transition error");
    } else if (error == -2) {
        SERIAL_MONITOR.println("parity error");
    } else if (error == -4) {
        SERIAL_MONITOR.println("invalid end sequence error");
    } else if (error == -8) {
        SERIAL_MONITOR.println("coax_buffered_rx overflow error");
    } else {
        SERIAL_MONITOR.print("unknown ");
        SERIAL_MONITOR.print(error);
        SERIAL_MONITOR.println("error");
    }
}

void testReadRegister()
{
    SERIAL_MONITOR.println();

    SERIAL_MONITOR.println("REGISTERS");

    uint8_t status = coax.readRegister(COAX_REGISTER_STATUS);

    SERIAL_MONITOR.print("  Status = ");
    SERIAL_MONITOR.println(status);

    SERIAL_MONITOR.print("    RX Error    = ");
    SERIAL_MONITOR.println(status & COAX_REGISTER_STATUS_RX_ERROR ? "Y" : "N");

    SERIAL_MONITOR.print("    RX Active   = ");
    SERIAL_MONITOR.println(status & COAX_REGISTER_STATUS_RX_ACTIVE ? "Y" : "N");

    SERIAL_MONITOR.print("    TX Complete = ");
    SERIAL_MONITOR.println(status & COAX_REGISTER_STATUS_TX_COMPLETE ? "Y" : "N");

    SERIAL_MONITOR.print("    TX Active   = ");
    SERIAL_MONITOR.println(status & COAX_REGISTER_STATUS_TX_ACTIVE ? "Y" : "N");

    uint8_t control = coax.readRegister(COAX_REGISTER_CONTROL);

    SERIAL_MONITOR.print("  Control = ");
    SERIAL_MONITOR.println(control);

    SERIAL_MONITOR.print("    Loopback    = ");
    SERIAL_MONITOR.println(control & COAX_REGISTER_CONTROL_LOOPBACK ? "Y" : "N");

    SERIAL_MONITOR.print("    TX Parity   = ");
    SERIAL_MONITOR.println(control & COAX_REGISTER_CONTROL_TX_PARITY ? "Even" : "Odd");

    SERIAL_MONITOR.print("    RX Parity   = ");
    SERIAL_MONITOR.println(control & COAX_REGISTER_CONTROL_RX_PARITY ? "Even" : "Odd");
}

int transmitReceive(const uint16_t *transmitBuffer, const size_t transmitBufferCount, uint16_t *receiveBuffer, const size_t receiveBufferSize, const uint16_t receiveTimeout)
{
    uint8_t status = coax.readRegister(COAX_REGISTER_STATUS);

    if (status & COAX_REGISTER_STATUS_RX_ACTIVE) {
        return /*ERROR_TX_RECEIVER_ACTIVE*/ -1;
    }

    if (status & COAX_REGISTER_STATUS_RX_ERROR) {
        SERIAL_MONITOR.println("[transmitReceive] - found coax in error state...");

        coax.reset();
    }

    coaxInterruptState = COAX_INTERRUPT_STATE_IDLE;

    // Transmit
    int transmitCount = coax.transmit(transmitBuffer, transmitBufferCount);

    if (transmitCount < 0) {
        indicators.error();

        return transmitCount;
    } else {
        indicators.tx();
    }

    // Receive
    if (receiveTimeout > 0) {
        unsigned long startTime = millis();

        while (coaxInterruptState == COAX_INTERRUPT_STATE_IDLE) {
            // https://www.forward.com.au/pfod/ArduinoProgramming/TimingDelaysInArduino.html#unsigned
            if ((millis() - startTime) > receiveTimeout) {
                return /*ERROR_RX_TIMEOUT*/ -2;
            }
        }
    }

    while (!(coaxInterruptState == COAX_INTERRUPT_STATE_RECEIVED || coaxInterruptState == COAX_INTERRUPT_STATE_ERROR)) {
        // NOP...
    }

    int count;

    if (coaxInterruptState == COAX_INTERRUPT_STATE_RECEIVED) {
        count = coaxInterruptBufferCount > receiveBufferSize ? receiveBufferSize : coaxInterruptBufferCount;

        memcpy(receiveBuffer, coaxInterruptBuffer, count * sizeof(uint16_t));

        indicators.rx();
    } else if (coaxInterruptState == COAX_INTERRUPT_STATE_ERROR) {
        count = (-1) * coaxInterruptError;

        indicators.error();
    }

    coaxInterruptState = COAX_INTERRUPT_STATE_IDLE;

    return count;
}

// vvv

#define FRAME_END 0xc0
#define FRAME_ESCAPE 0xdb
#define FRAME_ESCAPE_END 0xdc
#define FRAME_ESCAPE_ESCAPE 0xdd

enum {
    FRAME_STATE_WAIT_START,
    FRAME_STATE_DATA,
    FRAME_STATE_ESCAPE
} frameState;

#define FRAME_BUFFER_SIZE (COAX_BUFFER_SIZE * 2) + 32

uint8_t frameBuffer[FRAME_BUFFER_SIZE];
int frameBufferCount = 0;

#define ERROR_INVALID_MESSAGE 1
#define ERROR_UNKNOWN_COMMAND 2

inline void slipWrite(const uint8_t byte)
{
    if (byte == FRAME_END) {
        SERIAL_INTERFACE.write((char) FRAME_ESCAPE);
        SERIAL_INTERFACE.write((char) FRAME_ESCAPE_END);
    } else if (byte == FRAME_ESCAPE) {
        SERIAL_INTERFACE.write((char) FRAME_ESCAPE);
        SERIAL_INTERFACE.write((char) FRAME_ESCAPE_ESCAPE);
    } else {
        SERIAL_INTERFACE.write((char) byte);
    }
}

void sendMessage(const uint8_t *buffer, const int bufferCount)
{
    SERIAL_INTERFACE.write((char) FRAME_END);

    // Write the length.
    slipWrite((uint8_t) (bufferCount >> 8));
    slipWrite((uint8_t) bufferCount);

    for (int index = 0; index < bufferCount; index++) {
        slipWrite(buffer[index]);
    }

    // Write the placeholder for checksum.
    slipWrite(0x00);
    slipWrite(0x00);

    SERIAL_INTERFACE.write((char) FRAME_END);

    SERIAL_INTERFACE.flush();
}

void sendErrorMessage(const uint8_t code, const char *description)
{
    uint8_t message[2 + 62 + 1] = { 0x02, code };
    int count = 2;

    if (description != NULL) {
        strncpy((char *) (message + 2), description, 62);

        count += strlen(description);
    }

    sendMessage(message, count);
}

#define COMMAND_RESET 0x01
#define COMMAND_TRANSMIT_RECEIVE 0x06

void handleResetCommand(const uint8_t *buffer, const int bufferCount)
{
    coax.reset();

    uint8_t response[] = { 0x01, 0x00, 0x00, 0x01 };

    sendMessage(response, 4);
}

void handleTransmitReceiveCommand(uint8_t *buffer, int bufferCount)
{
    if (bufferCount < 6) {
        sendErrorMessage(ERROR_INVALID_MESSAGE, "HANDLE_TXRX_BUFFER_COUNT_6");
        return;
    }

    uint16_t *transmitBuffer = (uint16_t *) (buffer + 2);
    uint16_t transmitBufferCount = (bufferCount - 6) / 2;

    uint16_t transmitRepeatCount = ((buffer[0] << 8) | buffer[1]) & 0x7fff;
    uint16_t transmitRepeatOffset = buffer[0] >> 7;

    uint16_t receiveBufferSize = (buffer[bufferCount - 4] << 8) | buffer[bufferCount - 3];
    uint16_t receiveTimeout = (buffer[bufferCount - 2] << 8) | buffer[bufferCount - 1];

    if (transmitBufferCount < 1) {
        sendErrorMessage(ERROR_INVALID_MESSAGE, "HANDLE_TXRX_TX_BUFFER_COUNT_1");
        return;
    }

    digitalWrite(GPIO_0_PIN, HIGH);

    // Expand the provided data if applicable.
    if (transmitRepeatCount > 1) {
        uint8_t *source = ((uint8_t *) transmitBuffer) + (transmitRepeatOffset * 2);
        uint8_t *destination = ((uint8_t *) transmitBuffer) + (transmitBufferCount * 2);

        uint16_t sourceCount = transmitBufferCount - transmitRepeatOffset;

        size_t length = sourceCount * 2;

        for (int index = 1; index < transmitRepeatCount; index++) {
            memcpy(destination, source, length);

            transmitBufferCount += sourceCount;

            destination += length;
        }
    }

    uint16_t *receiveBuffer = (uint16_t *) (buffer + 2);

    digitalWrite(GPIO_1_PIN, HIGH);

    bufferCount = transmitReceive(transmitBuffer, transmitBufferCount, receiveBuffer, receiveBufferSize, receiveTimeout);

    digitalWrite(GPIO_1_PIN, LOW);

    if (bufferCount < 0) {
        sendErrorMessage(100 + ((-1) * bufferCount), NULL);
        return;
    }

    // Send the response message.
    buffer[1] = 0x01;

    bufferCount = 1 + (bufferCount * 2);

    sendMessage(buffer + 1, bufferCount);

    digitalWrite(GPIO_0_PIN, LOW);
}

void handleEchoMessage(uint8_t *buffer, int bufferCount)
{
    SERIAL_MONITOR.println();
    SERIAL_MONITOR.print(bufferCount);
    SERIAL_MONITOR.println(" echo request.");
    sendMessage(buffer, bufferCount);
}

void handleMessage(uint8_t *buffer, int bufferCount)
{
    if (bufferCount < 1) {
        sendErrorMessage(ERROR_INVALID_MESSAGE, "HANDLE_MESSAGE_BUFFER_COUNT_1");
        return;
    }

    uint8_t command = buffer[0];

    if (command == 22) {
        handleEchoMessage(buffer + 1, bufferCount - 1);
    } else if (command == COMMAND_RESET) {
        handleResetCommand(buffer + 1, bufferCount - 1);
    } else if (command == COMMAND_TRANSMIT_RECEIVE) {
        handleTransmitReceiveCommand(buffer + 1, bufferCount - 1);
    } else {
        sendErrorMessage(ERROR_UNKNOWN_COMMAND, NULL);
    }
}

void handleFrame(uint8_t *buffer, int bufferCount)
{
    if (bufferCount < 4) {
        sendErrorMessage(ERROR_INVALID_MESSAGE, "HANDLE_FRAME_BUFFER_COUNT_4");
        return;
    }

    int count = (buffer[0] << 8) | buffer[1];

    if (bufferCount - 4 != count) {
        sendErrorMessage(ERROR_INVALID_MESSAGE, "HANDLE_FRAME_BUFFER_COUNT_MISMATCH");
        return;
    }

    handleMessage(buffer + 2, count);
}

// ^^^

void setup()
{
    indicators.begin();

    // vvv
    Serial1.begin(115200, SERIAL_8N1);

    SerialUSB.begin();
    // ^^^

    SERIAL_MONITOR.println("COAX");

    while (SERIAL_INTERFACE.available()) {
        SERIAL_INTERFACE.read();
    }

    frameState = FRAME_STATE_WAIT_START;

    // vvv
    pinMode(GPIO_0_PIN, OUTPUT);
    digitalWrite(GPIO_0_PIN, LOW);

    pinMode(GPIO_1_PIN, OUTPUT);
    digitalWrite(GPIO_1_PIN, LOW);
    // ^^^

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
    /*
    static uint16_t buffer[COAX_BUFFER_SIZE];

    if (coaxInterruptState == COAX_INTERRUPT_STATE_RECEIVED) {
        size_t count = coaxInterruptBufferCount;

        memcpy(buffer, coaxInterruptBuffer, count * sizeof(uint16_t));

        coaxInterruptState = COAX_INTERRUPT_STATE_IDLE;

        handleReceiveData(buffer, count);
    }

    if (coaxInterruptState == COAX_INTERRUPT_STATE_ERROR) {
        int error = coaxInterruptError;

        coaxInterruptState = COAX_INTERRUPT_STATE_IDLE;

        handleReceiveError(error);
    }
    */

    if (SERIAL_INTERFACE.available() > 0) {
        uint8_t byte = SERIAL_INTERFACE.read();

        if (frameState == FRAME_STATE_WAIT_START) {
            if (byte == FRAME_END) {
                frameState = FRAME_STATE_DATA;
            }
        } else if (frameState == FRAME_STATE_DATA) {
            if (byte == FRAME_END) {
                if (frameBufferCount > 0) {
                    handleFrame(frameBuffer, frameBufferCount);
                }

                frameBufferCount = 0;
                frameState = FRAME_STATE_WAIT_START;
            } else if (byte == FRAME_ESCAPE) {
                frameState = FRAME_STATE_ESCAPE;
            } else {
                // TODO: overflow...
                frameBuffer[frameBufferCount++] = byte;
            }
        } else if (frameState == FRAME_STATE_ESCAPE) {
            if (byte == FRAME_ESCAPE_END) {
                // TODO: overflow...
                frameBuffer[frameBufferCount++] = FRAME_END;
            } else if (byte == FRAME_ESCAPE_ESCAPE) {
                // TODO: overflow...
                frameBuffer[frameBufferCount++] = FRAME_ESCAPE;
            }

            frameState = FRAME_STATE_DATA;
        }
    }

    if (SERIAL_MONITOR.available()) {
        char input = SERIAL_MONITOR.read();

        if (input == 'R') {
            SERIAL_MONITOR.println("RESET");
            coax.reset();
        } else if (input == 'r') {
            testReadRegister();
        } else if (input == 't') {
            selfTest();
        }

        SERIAL_MONITOR.flush();
    }
}
