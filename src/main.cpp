#include "board.h"
#include "mavlink.h"
#include "task.h"
#include "main.h"

void setup() {
    board_init();
    init_tasks();
}

void loop() {
    delay(1000);
}


void init_tasks() {
    INIT_STATIC_TASK(telemetry_task, "telemetry", NULL, 0, 0);
}

TASK telemetry_task(TaskDescriptor_t* self) {
    self->last_wake = xTaskGetTickCount();

    while (true) {
        mavlink_send_messages();
        mavlink_process_incoming_messages();

        TASK_WAIT_MS(self, 500);
    }
}
