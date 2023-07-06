/****************************************************************************
http://retro.moe/unijoysticle2

Copyright 2021 Ricardo Quesada

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
****************************************************************************/

#include "sdkconfig.h"
#ifndef CONFIG_BLUEPAD32_PLATFORM_ARDUINO
#error "Must only be compiled when using Bluepad32 Arduino platform"
#endif  // !CONFIG_BLUEPAD32_PLATFORM_ARDUINO

#include <Arduino.h>
#include <Bluepad32.h>
#define TASK_BUFFER_SIZE configSUPPORT_STATIC_ALLOCATION

GamepadPtr myGamepads[BP32_MAX_GAMEPADS];
GamepadPtr myGamepad = myGamepads[0];

bool u, d, r, l, a, b, x, y, l1, l2, r1, r2, thumbL, thumbR, miscSystem, miscBack, miscHome;
int32_t axisX, axisY, axisRX, axisRY, brake, throttle;

// This callback gets called any time a new gamepad is connected.
// Up to 4 gamepads can be connected at the same time.
void onConnectedGamepad(GamepadPtr gp) {
    bool foundEmptySlot = false;
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myGamepads[i] == nullptr) {
            Console.printf("CALLBACK: Gamepad is connected, index=%d\n", i);
            // Additionally, you can get certain gamepad properties like:
            // Model, VID, PID, BTAddr, flags, etc.
            GamepadProperties properties = gp->getProperties();
            Console.printf("Gamepad model: %s, VID=0x%04x, PID=0x%04x\n", gp->getModelName(), properties.vendor_id,
                           properties.product_id);
            myGamepads[i] = gp;
            foundEmptySlot = true;
            break;
        }
    }
    if (!foundEmptySlot) {
        Console.println("CALLBACK: Gamepad connected, but could not found empty slot");
    }
}

void onDisconnectedGamepad(GamepadPtr gp) {
    bool foundGamepad = false;

    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myGamepads[i] == gp) {
            Console.printf("CALLBACK: Gamepad is disconnected from index=%d\n", i);
            myGamepads[i] = nullptr;
            foundGamepad = true;
            break;
        }
    }

    if (!foundGamepad) {
        Console.println("CALLBACK: Gamepad disconnected, but not found in myGamepads");
    }
}

int decimalToBinary(int num) {
    int result = 0;
    int power = 1;

    while (num > 0) {
        int bit = num % 2;
        result += bit * power;
        num /= 2;
        power *= 10;
    }
    return result;
}
TaskHandle_t consolePrintGamePadTaskHandle;
StackType_t consolePrintGamePadTaskStack[4 * 1024];
StaticTask_t consolePrintGamePadTaskBuffer[TASK_BUFFER_SIZE];
void consolePrintGamePadTask(void* pt) {
    TickType_t xStartTick = xTaskGetTickCount();
    while (1) {
        if (myGamepad && myGamepad->isConnected()) {
            Console.printf("_________________________________________________\n");
            Console.printf("|u:%9d|d:%9d|l:%9d|r:%9d|\n", u = myGamepad->u(), d = myGamepad->d(), l = myGamepad->l(),
                           r = myGamepad->r());

            Console.printf("|a:%9d|b:%9d|x:%9d|y:%9d|\n", a = myGamepad->a(), b = myGamepad->b(), x = myGamepad->x(),
                           y = myGamepad->y());

            Console.printf("|l1:%8d|thumbL:%4d|r1:%8d|thumbR:%4d|\n", l1 = myGamepad->l1(),
                           thumbL = myGamepad->thumbL(), r1 = myGamepad->r1(), thumbR = myGamepad->thumbR());

            Console.printf("|Back:%6d|System:%4d|Home:%6d|total:%3dKb|\n", miscBack = myGamepad->miscBack(),
                           miscSystem = myGamepad->miscSystem(), miscHome = myGamepad->miscHome(),
                           ESP.getHeapSize() / 1024);

            Console.printf("|brake:%5d|thrtl:%5d|Free:%4dKb|Task:%4dKb|\n", brake = myGamepad->brake(),
                           throttle = myGamepad->throttle(), ESP.getFreeHeap() / 1024,
                           uxTaskGetStackHighWaterMark(consolePrintGamePadTaskHandle) / 1024);

            Console.printf("|axisLX:%4d|axisLY:%4d|axisRX:%4d|axisRY:%4d|\n", axisX = myGamepad->axisX(),
                           axisY = myGamepad->axisY(), axisRX = myGamepad->axisRX(), axisRY = myGamepad->axisRY());

            Console.printf("_________________________________________________\n");
        }
        xTaskDelayUntil(&xStartTick, 1000);
    }
}

// Arduino setup function. Runs in CPU 1
void setup() {
    Console.printf("Firmware: %s\n", BP32.firmwareVersion());
    const uint8_t* addr = BP32.localBdAddress();
    Console.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);
    Serial1.begin(115200);
    Serial1.println("Serial1.begin(115200);");
    // Setup the Bluepad32 callbacks
    BP32.setup(&onConnectedGamepad, &onDisconnectedGamepad);

    consolePrintGamePadTaskHandle =
        xTaskCreateStaticPinnedToCore(consolePrintGamePadTask, "consolePrintGamePadTask", 4 * 1024, NULL, 1,
                                      consolePrintGamePadTaskStack, consolePrintGamePadTaskBuffer, 1);
    Console.printf("consolePrintGamePadTask Created");
}
// Arduino loop function. Runs in CPU 1
void loop() {
    BP32.update();
    myGamepad = myGamepads[0];
    vTaskDelay(10);
    // delay(1000);
}
