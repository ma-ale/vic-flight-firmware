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
    INIT_STATIC_TASK(telemetry_task, "telemetry", NULL, 1, 0);

    INIT_STATIC_TASK(telemetry_task, "flight_system", NULL, configMAX_PRIORITIES-2, 0);
}

TASK telemetry_task(TaskDescriptor_t* self) {
    self->last_wake = xTaskGetTickCount();

    while (true) {
        mavlink_send_messages();
        mavlink_process_incoming_messages();

        TASK_WAIT_MS(self, 1000);
    }
}

TASK flight_system_task(TaskDescriptor_t* self) {
    self->last_wake = xTaskGetTickCount();

    while (true) {
        TASK_WAIT_MS(self, 10);
    }
}
