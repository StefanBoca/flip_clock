// FreeRTOS - Version: Latest
// IRremote - Version: Latest

#define DEBUG

#include <Arduino_FreeRTOS.h>
#include <Stepper.h>
#include <queue.h>

/***** TINY RECEIVER MACROS *****/
// These need to come before the TinyIRReceiver include
#define NO_LED_FEEDBACK_CODE
#define TINY_RECEIVER_USE_ARDUINO_ATTACH_INTERRUPT
#define IR_INPUT_PIN 2

#include "TinyIRReceiver.hpp"

/***** STEPPER CONSTANTS *****/

#define STEPS_PER_REV 2048
#define STEPPER_SPEED 15

Stepper steppers[] = {Stepper(STEPS_PER_REV, 22, 26, 24, 28), Stepper(STEPS_PER_REV, 23, 27, 25, 29)};

/***** DISPLAY CONSTANTS *****/

/* sevent segment  pins A, B, C,  D, E, F,  G,  H */
const int SS_PINS[8] = {6, 7, 8, 9, 10, 11, 12, 13};

// seven segment character look up table
const bool SS_CHAR_LUT[10][8] = {
    /* pin A, B, C, D, E, F, G, H */
    /*0*/ {1, 1, 1, 1, 1, 1, 0, 0},
    /*1*/ {0, 1, 1, 0, 0, 0, 0, 0},
    /*2*/ {1, 1, 0, 1, 1, 0, 1, 0},
    /*3*/ {1, 1, 1, 1, 0, 0, 1, 0},
    /*4*/ {0, 1, 1, 0, 0, 1, 1, 0},
    /*5*/ {1, 0, 1, 1, 0, 1, 1, 0},
    /*6*/ {1, 0, 1, 1, 1, 1, 1, 0},
    /*7*/ {1, 1, 1, 0, 0, 0, 0, 0},
    /*8*/ {1, 1, 1, 1, 1, 1, 1, 0},
    /*9*/ {1, 1, 1, 1, 0, 1, 1, 0},
};

/***** DISPLAY STATE *****/

int currentDigit = 0;
bool power = true;

/***** DISPLAY FUNCTIONS *****/

void clearDisplay() {
    for (int i = 0; i < 8; i++) { digitalWrite(SS_PINS[i], LOW); }
}

void writeDisplay(int n) {
    if (n < 0 || n > 9) {
        // n out of range
        clearDisplay();
    }

    for (int i = 0; i < 8; i++) { digitalWrite(SS_PINS[i], SS_CHAR_LUT[n][i]); }
    currentDigit = n;
    power = 1;
}

void toggleDisplayPower() {
    if (power)
        clearDisplay();
    else
        writeDisplay(currentDigit);
    power = !power;
}

/***** IR INTERRUPT ISR *****/

QueueHandle_t irQueue;

struct IrData {
    uint16_t address;
    uint16_t command;
    bool isRepeat;
};

void handleReceivedTinyIRData(uint16_t address, uint8_t command, bool isRepeat) {
#ifdef DEBUG
    Serial.print(F("A=0x"));
    Serial.print(address, HEX);
    Serial.print(F(" C=0x"));
    Serial.print(command, HEX);
    Serial.print(F(" R="));
    Serial.print(isRepeat);
    Serial.println();
#endif
    BaseType_t xHigherPriorityTaskWoken;
    struct IrData irData = {address, command, isRepeat};
    xQueueSendFromISR(irQueue, &irData, &xHigherPriorityTaskWoken);
}

/***** TASKS *****/

void TaskHandleInput(void* pvParameters) {
    (void)pvParameters;

    IrData irData;
    for (;;) {
        if (xQueueReceive(irQueue, &irData, portMAX_DELAY) == pdPASS) {
            if (irData.address == 0 && irData.isRepeat == 0) {
                switch (irData.command) {
                    case 0x16: /*0*/ writeDisplay(0); break;
                    case 0x0C: /*1*/ writeDisplay(1); break;
                    case 0x18: /*2*/ writeDisplay(2); break;
                    case 0x5E: /*3*/ writeDisplay(3); break;
                    case 0x08: /*4*/ writeDisplay(4); break;
                    case 0x1C: /*5*/ writeDisplay(5); break;
                    case 0x5A: /*6*/ writeDisplay(6); break;
                    case 0x42: /*7*/ writeDisplay(7); break;
                    case 0x52: /*8*/ writeDisplay(8); break;
                    case 0x4A: /*9*/ writeDisplay(9); break;
                    case 0x45: /*power*/ toggleDisplayPower(); break;
                }
            }
        }

        vTaskDelay(10);
    }
}

void TaskTestStep(void* pvParameters) {
    (void)pvParameters;

    for (;;) {
        steppers[0].step(STEPS_PER_REV / 128);
        if (uxQueueMessagesWaiting(irQueue) > 0) {
            // This is necessary otherwise the context will never switch to input handling
            vTaskDelay(1);
        }
    }
}

/***** MAIN FUNCTIONS *****/

void setup() {
    Serial.begin(115200);

    // Set all SS pins to output
    for (int p : SS_PINS) { pinMode(p, OUTPUT); }

    steppers[0].setSpeed(10);

    irQueue = xQueueCreate(10, sizeof(IrData));

    xTaskCreate(TaskHandleInput,
                "HandleInput",
                128, // Stack size
                NULL,
                1, // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
                NULL);

    xTaskCreate(TaskTestStep,
                "TestStep",
                128, // Stack size
                NULL,
                3, // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
                NULL);

    initPCIInterruptForTinyReceiver();
}

void loop() {
    // Do nothing, everything is handled by tasks
}
