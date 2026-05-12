#pragma once

/**
 ** Authors: Enrico Lumetti
 ** Simple mavlink layer for telemetry
 */

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

int mavlink_process_incoming_messages();
void mavlink_send_messages();

#ifdef __cplusplus
}
#endif
