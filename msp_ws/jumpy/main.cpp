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


/* DEBUG
 * check timing with updated DCO config
 * check timing with crystal
 * check sense freq
 * check control freq
 * check transmit freq
 * check valve sequence - have to implement in ROS first
 * check trajectory dump
 * how to separate dump from transmit????
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
    int idx_sense = 0; // sensor read index
    int n_sense = 0; // number of sensor reads
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
                n_sense = idx_sense; // save number of sensor reads
                idx_sense = 0; // reset sensor read index
                idx_seq = 0; // reset valve time sequence index
                t_valve_seq = 0; // reset trajectory time
            }
        }


        // dump trajectory data via UART
        if (flag_dump) {
            for (int i = 0; i < n_sense; i++) {
                sendData(data_traj[i], 6);
            }
            flag_dump = 0;
        }


        // sense, control, receive, transmit
        if (flag_sense) { // read sensors
            sensorUpdate(idx_sense);
            flag_sense = 0; // clear flag
            idx_sense = idx_sense + flag_valve_seq; // increment sensor read index if running valve time seq
        }
        if (flag_control) { // update control
            controlUpdate();
            flag_control = 0; // clear flag
        }
        if (flag_receive) { // process received data
            receiveData();
            flag_receive = 0; // clear flag
        }
        if (flag_transmit & !flag_valve_seq & !flag_dump) { // transmit data if not running traj
            sendData(uart_tx, 5);
            flag_transmit = 0; // clear flag
            MAP_GPIO_toggleOutputOnPin(GPIO_PORT_P1, GPIO_PIN0); // toggle red LED
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


extern "C" void TA3_0_IRQHandler(void) {
    flag_transmit = 1; // set flag to get sensor values
    MAP_Timer_A_clearCaptureCompareInterrupt(TIMER_A3_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_0); // clear interrupt flag
}


extern "C" void SysTick_Handler(void) {
    t_valve_seq = t_valve_seq + 0.0001f;
}


