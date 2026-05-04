#include "board.h"
#include "mavlink.h"

void setup() {
    board_init();
}

void loop() {
    mavlink_process_incoming_message(MAVLINK_COMM_0);
    mavlink_send_messages(MAVLINK_COMM_0);

    delay(1000);
}
