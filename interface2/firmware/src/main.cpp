// Copyright (c) 2020, Andrew Kay
//
// Permission to use, copy, modify, and/or distribute this software for any
// purpose with or without fee is hereby granted, provided that the above
// copyright notice and this permission notice appear in all copies.
//
// THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
// WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
// MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
// ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
// WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
// ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
// OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

#include <Arduino.h>

#include "SPICoaxTransceiver.h"

#define CS_PIN 10

SPICoaxTransceiver transceiver(CS_PIN);

void setup()
{
    // Configure serial port.
    Serial.begin(115200);

    while (Serial.available() > 0) {
        Serial.read();
    }

    // Configure the transceiver.
    transceiver.begin();
}

uint16_t buffer[1024];

void loop()
{
    if (Serial.available()) {
        char command = Serial.read();

        if (command == '1') {
            Serial.println("RX");

            int count = transceiver.receive(buffer, 1024);

            if (count == RX_ERROR_LOSS_OF_MID_BIT_TRANSITION) {
                Serial.println("\tLOSS OF MID BIT TRANSITION ERROR");
            } else if (count == RX_ERROR_PARITY) {
                Serial.println("\tPARITY ERROR");
            } else if (count == RX_ERROR_INVALID_END_SEQUENCE) {
                Serial.println("\tINVALID END SEQUENCE ERROR");
            } else if (count == RX_ERROR_OVERFLOW) {
                Serial.println("\tOVERFLOW ERROR");
            } else if (count == 0) {
                Serial.println("\tEMPTY");
            } else {
                Serial.print("\t");
                Serial.println(count);
            }
        }
    }
}
