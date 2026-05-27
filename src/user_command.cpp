#include "user_command.hpp"

UserCommandTask::UserCommandTask(DriveTrainTask *drive_train_task)
    : SchedulerTask("user_command_task"), drive_train_task(drive_train_task) {}

const uint8_t COMMAND_HEADER[10] = {
    0x99, 0x88, 0x77, 0x66, 0x55, 0x44, 0x33, 0x22, 0x11, 0x00,
};

struct __attribute__((packed)) CommandPacket {
    int8_t left_command;
    int8_t right_command;
};

static uint8_t SERIAL_MEMORY[200];

void UserCommandTask::setup() { Serial7.addMemoryForRead(SERIAL_MEMORY, sizeof(SERIAL_MEMORY)); }

#define COMMAND_TIMEOUT 1000

void UserCommandTask::loop() {
    if (Serial7.available() < sizeof(COMMAND_HEADER) + sizeof(CommandPacket)) {
        if (millis() - last_contact > COMMAND_TIMEOUT) {
            drive_train_task->set_commands(0.0, 0.0);
        }

        return;
    }

    for (int i = 0; i < sizeof(COMMAND_HEADER); i++) {
        uint8_t c = Serial7.read();

        if (c != COMMAND_HEADER[i]) {
            while (Serial7.peek() != COMMAND_HEADER[0] && Serial7.available() > 0) {
                Serial7.read();
            }

            return;
        }
    }

    last_contact = millis();

    CommandPacket packet;

    Serial7.readBytes((char *)&packet, sizeof(CommandPacket));

    drive_train_task->set_commands((float)packet.left_command / 100.0,
                                   (float)packet.right_command / 100.0);
}