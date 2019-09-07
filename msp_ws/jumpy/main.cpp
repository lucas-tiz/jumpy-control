//*****************************************************************************
//
// MSP432 main.c - jumping robot: 4 joints
//
// Lucas Tiziani
// 7 September 2019
//
//****************************************************************************

/* DEBUG:
 * - test sensing frequency - figure out max freq
 * - test control frequency
 * - test comms frequency
 * - test 2x pin control for each valve
 */

/* TODO:
 * - add joint potentiometer code (initialization needed?)
 * - append f to floats
 */


#include "header.h"


void main(void) {
    MAP_WDT_A_holdTimer(); // hold the watchdog timer (stop from running)
    MAP_Interrupt_disableMaster(); // disable interrupts
    MAP_FPU_enableModule(); // enable floating-point unit

    configClocks();
    configValvePins();
    configGpioPins();
    configTimers();
    configAnalog();
    configUart();

    startTimers();
    MAP_Interrupt_enableMaster(); // enable interrupts

    int sendDataCount = 0;
    while(1) {
        if (sensorFlag) {
            sensorUpdate(); // get sensor data
            sensorFlag = 0; // clear sensor flag
            sendDataCount++; // increment sensor flag

            if (sendDataCount == 10) { // 100 Hz
                sendData(5); // send sensor data
                sendDataCount = 0; // reset sensor flag
                MAP_GPIO_toggleOutputOnPin(GPIO_PORT_P1, GPIO_PIN0); // toggle red LED
            }
        }
        if (controlFlag) {
            controlUpdate(); // update control
            controlFlag = 0; // clear control flag
        }
        if (updateValuesFlag) {
            updateValues(); // update pressure setpoint values
            updateValuesFlag = 0; // clear values update flag
        }

        // turn on LED to indicate test start as soon as at least one pressure is set
        if ((pres_des[0] <= 0) | (pres_des[1] <= 0) | (pres_des[2] <= 0) | (pres_des[3] <= 0)) {
            MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN2);
        }
        else {
            MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN2); // turn on blue LED
        }
    }
}

