//*****************************************************************************
//
// MSP432 main.c - jumping robot: 4 joints
//
// Lucas Tiziani
// 7 September 2019
//
//****************************************************************************

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

    int idx_seq = 0; // valve sequence index
    int n_sense = 0;
    while(1) {


        //DEBUG
        if (t_valve_seq >= 1.0) {
            MAP_GPIO_toggleOutputOnPin(GPIO_PORT_P3, GPIO_PIN7); // toggle debug pin
            t_valve_seq = 0;
        }


        // run valve time sequence
        if (flag_valve_seq) {
            MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN3); // turn on external LED
            if (t_valve_seq >= valve_seq[idx_seq][0]) { // if at next time in sequence
                std::copy(valve_seq[idx_seq], valve_seq[idx_seq]+4, pres_des); // update pressure setpoints
                controlUpdate(); // update control
                idx_seq++; // increment index
            }
            if (idx_seq = len_valve_seq) { // reset at end of sequence
                MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN3); // turn off external LED
                flag_valve_seq = 0; // reset flag
                idx_seq = 0; // reset valve time sequence index
                t_valve_seq = 0; // reset trajectory time
            }
        }


        if (flag_sense) {
            sensorUpdate(); // get sensor data
            flag_sense = 0; // clear sensor flag
            n_sense++; // increment sensor flag

//            if (n_sense == 5) { // 100 Hz
//                sendData(5); // send sensor data
//                n_sense = 0; // reset sensor flag
//                MAP_GPIO_toggleOutputOnPin(GPIO_PORT_P1, GPIO_PIN0); // toggle red LED
//            }
        }
        if (flag_control) {
            controlUpdate(); // update control
            flag_control = 0; // clear control flag
        }
        if (flag_receive) {
            receiveData(); // receive data
            flag_receive = 0; // clear data receive flag
        }


        // manually vent all valves if button 1 pushed
        if (MAP_GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN1) == 0) {
            for (int idx_valve = 0; idx_valve < 4; idx_valve++) {
                pres_des[idx_valve] = 0;
                MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN1); // turn on green LED
            }
        }
        // manually seal all valves if button 2 pushed
        if (MAP_GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN4) == 0) {
            for (int idx_valve = 0; idx_valve < 4; idx_valve++) {
                pres_des[idx_valve] = -1;
                MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN1); // turn off green LED
            }
        }
    }
}


extern "C" void SysTick_Handler(void) {
    t_valve_seq = t_valve_seq + 0.0001;
}


