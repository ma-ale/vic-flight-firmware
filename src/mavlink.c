#include "board.h"
#include "mavlink.h"
#include <mavlink/common/mavlink.h>

static mavlink_status_t status;
static MAV_MODE_FLAG mode_flag = 0;
static MAV_STATE system_status = MAV_STATE_STANDBY;

bool mavlink_process_incoming_message(int chan) {
    uint8_t byte;
    mavlink_message_t msg;
    while (board_telemetry_read_byte(&byte)) {
        if (mavlink_parse_char(chan, byte, &msg, &status)) {
            board_led_set(true);
            board_sleep_ms(20);
            board_led_set(false);

            return true;
        }
    }

    return false;
}


void mavlink_send_messages(int chan) {
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];

    mavlink_message_t msg;
    mavlink_heartbeat_t hb = {
        .type = MAV_TYPE_FIXED_WING,
        .autopilot = MAV_AUTOPILOT_INVALID,
        .base_mode = mode_flag,
        .custom_mode = 0,
        .system_status = system_status,
    };
    mavlink_msg_heartbeat_encode_chan(
        10, MAV_COMP_ID_ONBOARD_COMPUTER, MAVLINK_COMM_0, &msg, &hb);

    const int len = mavlink_msg_to_send_buffer(buffer, &msg);
    board_telemetry_write_buffer(buffer, len);
}
