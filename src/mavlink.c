#include "board.h"
#include "mavlink.h"
#include <mavlink/common/mavlink.h>

#define SYSTEM_ID 10

static mavlink_status_t status;
static MAV_MODE_FLAG mode_flag = 0;
static MAV_STATE system_status = MAV_STATE_STANDBY;

static void send_message(const mavlink_message_t* msg);
static void send_heartbeat_message();
static void send_sys_status();
static void send_gps_raw_int();
static void send_battery_status();
static void send_command_ack(uint16_t command, MAV_RESULT result, uint8_t target_sys, uint8_t target_comp);
static void send_autopilot_version();
static void process_message(const mavlink_message_t* msg);
static void process_int_command(const mavlink_command_int_t* cmd);
static void process_long_command(const mavlink_command_long_t* cmd);
static void process_cmd_request(const mavlink_command_long_t* cmd);
static void process_param_request_list();

int mavlink_process_incoming_messages() {
    uint8_t byte;
    mavlink_message_t msg;
    int num_processed = 0;
    while (board_telemetry_read_byte(&byte)) {
        if (mavlink_parse_char(MAVLINK_COMM_0, byte, &msg, &status)) {
            process_message(&msg);
            ++num_processed;
        }
    }

    return num_processed;
}


void mavlink_send_messages() {
    send_heartbeat_message();
    send_sys_status();
    send_gps_raw_int();
    send_battery_status();
}


static uint8_t tx_buffer[MAVLINK_MAX_PACKET_LEN];

void send_message(const mavlink_message_t* msg) {
    const int len = mavlink_msg_to_send_buffer(tx_buffer, msg);
    board_telemetry_write_buffer(tx_buffer, len);
}

void send_heartbeat_message() {
    mavlink_message_t msg;
    mavlink_heartbeat_t hb = {
        .type = MAV_TYPE_FIXED_WING,
        .autopilot = MAV_AUTOPILOT_GENERIC,
        .base_mode = mode_flag,
        .custom_mode = 0,
        .system_status = system_status,
    };
    mavlink_msg_heartbeat_encode_chan(
        SYSTEM_ID, MAV_COMP_ID_AUTOPILOT1, MAVLINK_COMM_0, &msg, &hb);
    send_message(&msg);
}

void send_sys_status() {
    mavlink_message_t msg;

    const uint32_t sensors = MAV_SYS_STATUS_SENSOR_BATTERY | MAV_SYS_STATUS_SENSOR_GPS;
                           //| MAV_SYS_STATUS_SENSOR_3D_GYRO
                           //| MAV_SYS_STATUS_SENSOR_3D_ACCEL
                           //| MAV_SYS_STATUS_SENSOR_GPS
                           //| MAV_SYS_STATUS_AHRS;

    mavlink_sys_status_t ss = {
        .onboard_control_sensors_present = sensors,
        .onboard_control_sensors_enabled = sensors,
        .onboard_control_sensors_health  = sensors,
        .load             = 0,
        .voltage_battery  = 12000, // mV
        .current_battery  = 0,     //
        .battery_remaining = 80,   // 0-100
        .drop_rate_comm   = 0,
        .errors_comm      = 0,
    };
    mavlink_msg_sys_status_encode_chan(
        SYSTEM_ID, MAV_COMP_ID_AUTOPILOT1, MAVLINK_COMM_0, &msg, &ss);
    send_message(&msg);
}

void send_gps_raw_int() {
    mavlink_message_t msg;
    mavlink_gps_raw_int_t gps = {
        .time_usec         = 0,
        .lat               = 444949000,   // 44.4949 N (Bologna)
        .lon               = 113426000,   // 11.3426 E
        .alt               = 50000,       // 50 m in mm
        .eph               = 100,         // HDOP 1.0
        .epv               = 150,         // VDOP 1.5
        .vel               = 0,
        .cog               = UINT16_MAX,
        .fix_type          = GPS_FIX_TYPE_3D_FIX,
        .satellites_visible = 10,
    };
    mavlink_msg_gps_raw_int_encode_chan(
        SYSTEM_ID, MAV_COMP_ID_AUTOPILOT1, MAVLINK_COMM_0, &msg, &gps);
    send_message(&msg);
}

void send_battery_status() {
    mavlink_message_t msg;
    mavlink_battery_status_t bs = {
        .id                = 0,
        .battery_function  = MAV_BATTERY_FUNCTION_ALL,
        .type              = MAV_BATTERY_TYPE_LIPO,
        .temperature       = INT16_MAX,   // sconosciuta
        .voltages          = {12000, UINT16_MAX, UINT16_MAX, UINT16_MAX,
                               UINT16_MAX, UINT16_MAX, UINT16_MAX, UINT16_MAX,
                               UINT16_MAX, UINT16_MAX},
        .current_battery   = -1,          // non disponibile
        .current_consumed  = -1,          // non disponibile
        .energy_consumed   = -1,          // non disponibile
        .battery_remaining = 80,
    };
    mavlink_msg_battery_status_encode_chan(
        SYSTEM_ID, MAV_COMP_ID_AUTOPILOT1, MAVLINK_COMM_0, &msg, &bs);
    send_message(&msg);
}

void send_command_ack(uint16_t command, MAV_RESULT result, uint8_t target_sys, uint8_t target_comp) {
    mavlink_message_t msg;
    mavlink_command_ack_t ack = {
        .command = command,
        .result = result,
        .target_system = target_sys,
        .target_component = target_comp,
    };
    mavlink_msg_command_ack_encode_chan(
        SYSTEM_ID, MAV_COMP_ID_AUTOPILOT1, MAVLINK_COMM_0, &msg, &ack);
    send_message(&msg);
}

void send_autopilot_version() {
    mavlink_message_t msg;
    mavlink_autopilot_version_t av = {
        .capabilities = MAV_PROTOCOL_CAPABILITY_MAVLINK2,
        .flight_sw_version = 0,
        .middleware_sw_version = 0,
        .os_sw_version = 0,
        .board_version = 0,
        .vendor_id = 0,
        .product_id = 0,
        .uid = 0,
    };
    mavlink_msg_autopilot_version_encode_chan(
        SYSTEM_ID, MAV_COMP_ID_AUTOPILOT1, MAVLINK_COMM_0, &msg, &av);
    send_message(&msg);
}

void process_message(const mavlink_message_t* msg) {
    switch (msg->msgid) {
        case MAVLINK_MSG_ID_COMMAND_INT: {
            mavlink_command_int_t cmd;
            mavlink_msg_command_int_decode(msg, &cmd);
            process_int_command(&cmd);
            break;
        }
        case MAVLINK_MSG_ID_COMMAND_LONG: {
            mavlink_command_long_t cmd;
            mavlink_msg_command_long_decode(msg, &cmd);
            process_long_command(&cmd);
            break;
        }
        case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
            process_param_request_list();
            break;
        case MAVLINK_MSG_ID_PARAM_REQUEST_READ:
            break;
    }
}

void process_param_request_list() {
    mavlink_message_t msg;
    mavlink_param_value_t pv = {
        .param_count = 0,
        .param_index = 0,
        .param_value = 0,
        .param_type  = MAV_PARAM_TYPE_REAL32,
        .param_id    = "",
    };
    mavlink_msg_param_value_encode_chan(
        SYSTEM_ID, MAV_COMP_ID_AUTOPILOT1, MAVLINK_COMM_0, &msg, &pv);
    send_message(&msg);
}

void process_int_command(const mavlink_command_int_t* cmd) {

}

void process_long_command(const mavlink_command_long_t* cmd) {
    switch (cmd->command) {
        case MAV_CMD_REQUEST_MESSAGE:
            process_cmd_request(cmd);
            break;
    }
}

void process_cmd_request(const mavlink_command_long_t* cmd) {
    switch ((uint32_t) cmd->param1) {
        case MAVLINK_MSG_ID_AUTOPILOT_VERSION:
            send_command_ack(cmd->command, MAV_RESULT_ACCEPTED, cmd->target_system, cmd->target_component);
            send_autopilot_version();
            break;
        default:
            send_command_ack(cmd->command, MAV_RESULT_DENIED, cmd->target_system, cmd->target_component);
            break;
    }
}
