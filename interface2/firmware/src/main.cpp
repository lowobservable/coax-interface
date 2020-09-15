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
#define INTERRUPT_PIN 20

SPICoaxTransceiver transceiver(CS_PIN);

// vvv
#define RX_BUFFER_SIZE 256

uint16_t rx_buffer[RX_BUFFER_SIZE];

void handleInterrupt()
{

    Serial.print("[");

    int count;
    uint8_t status;

    do {
        count = transceiver.receive(rx_buffer, RX_BUFFER_SIZE);

        if (count < 0) {
            if (count == RX_ERROR_LOSS_OF_MID_BIT_TRANSITION) {
                Serial.print("LOSS OF MID BIT TRANSITION ERROR");
            } else if (count == RX_ERROR_PARITY) {
                Serial.print("PARITY ERROR");
            } else if (count == RX_ERROR_INVALID_END_SEQUENCE) {
                Serial.print("INVALID END SEQUENCE ERROR");
            } else if (count == RX_ERROR_OVERFLOW) {
                Serial.print("OVERFLOW ERROR");
            } else {
                Serial.print("UNKNOWN ERROR");
            }

            break;
        } else if (count == 0) {
            Serial.print(".");
        } else {
            Serial.print(count);
            Serial.print(" ");
        }

        status = transceiver.readRegister(COAX_REGISTER_STATUS);
    } while (status & COAX_REGISTER_STATUS_RX_ACTIVE || count == RX_BUFFER_SIZE);

    Serial.println("]");
}
// ^^^

void setup()
{
    // Configure serial port.
    Serial.begin(115200);

    while (Serial.available() > 0) {
        Serial.read();
    }

    // Configure the transceiver.
    transceiver.begin();

    pinMode(INTERRUPT_PIN, INPUT);

    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), handleInterrupt, RISING);
}

uint16_t buffer[1024];

void loop()
{
    if (Serial.available()) {
        char command = Serial.read();

        if (command == 's') {
            Serial.println("READ_REGISTER STATUS");
            uint8_t value = transceiver.readRegister(COAX_REGISTER_STATUS);

            Serial.print("\t");
            Serial.println(value, HEX);
        } else if (command == 'x') {
            Serial.println("X");
            transceiver.receive(rx_buffer, RX_BUFFER_SIZE);
        }
    }
}
