#pragma once

#include <mavlink/common/mavlink.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

bool mavlink_process_incoming_message(int chan);
void mavlink_send_messages(int chan);

#ifdef __cplusplus
}
#endif
