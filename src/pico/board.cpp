#include "board.h"
#include <Arduino.h>

void board_init() {
    Serial.begin(115200);
    pinMode(LED_BUILTIN, OUTPUT);
}

void board_sleep_ms(uint32_t ms) {
    delay(ms);
}

void board_led_set(bool value) {
    digitalWrite(LED_BUILTIN, value);
}

bool board_telemetry_read_byte(uint8_t* byte) {
    if (Serial.available() <= 0)
        return false;

    *byte = Serial.read();
    return true;
}

void board_telemetry_write_buffer(const uint8_t* buf, int32_t len) {
    Serial.write(buf, len);
}
