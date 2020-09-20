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

    delay(500);

    indicators.setStatus(RUNNING);
}

void loop()
{
    if (SerialUSB.available()) {
        char input = SerialUSB.read();

        if (input == 't') {
            testReadRegister();
        }

        SerialUSB.flush();
    }
}
