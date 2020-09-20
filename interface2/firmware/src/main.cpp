#include <Arduino.h>

#include <Indicators.h>
#include <ICE40.h>
#include <SPICoaxTransceiver.h>

#include <pins.h>

const uint8_t ice40Bitstream[] = {
#include "bitstream.hex"
};

Indicators indicators(LED_STATUS_PIN, LED_TX_PIN, LED_RX_PIN, LED_ERROR_PIN);

ICE40 ice40(ICE40_CS_PIN, ICE40_SCK_PIN, ICE40_SDI_PIN, ICE40_CRESET_PIN,
        ICE40_CDONE_PIN);

SPICoaxTransceiver coax(ICE40_CS_PIN);

void testReadRegister()
{
    SerialUSB.println("TEST READ REGISTER");

    uint8_t value = coax.readRegister(COAX_REGISTER_STATUS);

    SerialUSB.print("\tvalue = ");
    SerialUSB.println(value);
}

#define COAX_BUFFER_SIZE 1 + (3696 * 2)

uint16_t coaxBuffer[COAX_BUFFER_SIZE];

volatile bool inTheCoaxInterrupt = false;
volatile int coaxInterruptResult = 0;

void handleCoaxInterrupt()
{
    if (inTheCoaxInterrupt) {
        // XXX: okay, is this correct... this isn't the actual overflow sitation
        return;
    }

    inTheCoaxInterrupt = true;

    coaxInterruptResult = 1;

    inTheCoaxInterrupt = false;
}

void testReceive()
{
    SerialUSB.println("TEST RECEIVE");

    int result = coax.receive(coaxBuffer, COAX_BUFFER_SIZE);

    if (result < 0) {
        SerialUSB.print("\tERROR ");
        SerialUSB.println(result);
    } else if (result == 0) {
        SerialUSB.println("\tEMPTY");
    } else {
        SerialUSB.print("\t");
        SerialUSB.print(result);
        SerialUSB.println(" words");
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

void loop()
{
    if (coaxInterruptResult != 0) {
        // XXX: this bit should be as fast as possible...

        SerialUSB.println("INTERRUPT");
        SerialUSB.print("\tresult = ");
        SerialUSB.println(coaxInterruptResult);

        coaxInterruptResult = 0;

        SerialUSB.flush();
    } else if (SerialUSB.available()) {
        char input = SerialUSB.read();

        if (input == '1') {
            testReadRegister();
        } else if (input == '2') {
            testReceive();
        }

        SerialUSB.flush();
    }
}
