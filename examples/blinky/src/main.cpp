/**
 * PIO-Air105 — Blinky + Serial Echo Example
 *
 * Blinks the onboard RED LED at 1 Hz, prints status to Serial,
 * and echoes back any received characters.
 * Demonstrates basic Arduino API on the Air105 MCU.
 */

#include <Arduino.h>

void setup() {
    Serial.begin(115200);
    Serial.println("PIO-Air105 Blinky + Echo Example");
    Serial.println("Type something to test Serial.read()");

    pinMode(LED_BUILTIN, OUTPUT);
}

unsigned long lastBlink = 0;
bool ledState = false;

void loop() {
    /* Blink LED at 1 Hz (non-blocking) */
    if (millis() - lastBlink >= 500) {
        lastBlink = millis();
        ledState = !ledState;
        digitalWrite(LED_BUILTIN, ledState ? HIGH : LOW);
    }

    /* Echo any received serial data */
    while (Serial.available()) {
        int c = Serial.read();
        Serial.print("Echo: 0x");
        if (c < 0x10) Serial.print('0');
        Serial.print(c, HEX);
        Serial.print(" '");
        Serial.write((char)c);
        Serial.println("'");
    }
}
