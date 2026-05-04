#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <Arduino.h>

#ifdef __cplusplus
extern "C" {
#endif

void board_init();

void board_sleep_ms(uint32_t ms);

void board_led_set(bool value);

bool board_telemetry_read_byte(uint8_t* byte);
void board_telemetry_write_buffer(const uint8_t* buf, int32_t len);


#ifdef __cplusplus
}
#endif
