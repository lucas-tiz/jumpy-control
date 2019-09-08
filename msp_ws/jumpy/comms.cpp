#include "header.h"
#include <vector>
#include <array>
#include <algorithm>
#include <iterator>
#include "byte_stuff.hpp"


// send data over UART
void sendData(int n_float_tx) { //TODO: restructure function to take number of floats, etc.
    // set up data structures
//    int n_float_tx = 15; // number of floats to transmit
    int n8_tx = n_float_tx*4; // number of data bytes to transmit
    uint8_t arr8_unstuff_tx[252]; //[n8_tx]; // array for unstuffed data
    uint8_t arr8_stuff_tx[256]; // array for stuffed data

    //DEBUG:
//    uart_tx[1] = pres_des[0];
//    uart_tx[2] = pres_des[1];
//    uart_tx[3] = pres_des[2];
//    uart_tx[4] = pres_des[3];


    // separate, stuff, and send data
    SeparateArr(uart_tx, n_float_tx, arr8_unstuff_tx);  // separate floats into bytes
    StuffArr(arr8_unstuff_tx, n8_tx, arr8_stuff_tx); // create stuffed byte packet
    for (int i = 0; i < (n8_tx+2); i++) { // loop over data bytes + 2 stuff bytes
        MAP_UART_transmitData(EUSCI_A0_BASE, arr8_stuff_tx[i]); // transmit byte
        while (MAP_UART_getInterruptStatus(EUSCI_A0_BASE, EUSCI_A_UART_TRANSMIT_INTERRUPT_FLAG)
                != EUSCI_A_UART_TRANSMIT_INTERRUPT_FLAG); // wait for transmission completion
    }
}


// receive data over UART: eUSCI A module interrupt routine
void EUSCIA0_IRQHandler(void) {
    MAP_Interrupt_disableMaster(); // disable all interrupts

    static uint8_t arr8_stuff_rx[256]; // array for stuffed data
    static uint8_t arr8_unstuff_rx[252]; // array for unstuffed data
    static float arr_float_cat_rx[63]; // array for concatenated data
    static uint8_t buf_rx; // byte receive buffer
    static int n8_rx = 0; // number of bytes received in packet

    if (MAP_UART_getInterruptStatus(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG)
            == EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG) { // if receive interrupt flag

        buf_rx = MAP_UART_receiveData(EUSCI_A0_BASE); // access received data by reading RXBUF register; flag automatically cleared
        if (buf_rx == 0) { // if 0 received
            arr8_stuff_rx[n8_rx] = buf_rx; // add byte to array
            n8_rx++; // increment number of bytes in packet

            UnstuffArr(arr8_stuff_rx, n8_rx, arr8_unstuff_rx); // unstuff data in packet (auto clears unstuff vector)
            ConcatArr(arr8_unstuff_rx, n8_rx-2, arr_float_cat_rx); // concatenate bytes into uint16_ts (auto clears cat vector)
            int n_float_rx = (n8_rx-2)/4; // number of floats received
            std::copy(arr_float_cat_rx, arr_float_cat_rx + n_float_rx, uart_rx); // add received data into global array
            n8_rx = 0; // reset packet length
            updateValuesFlag = 1; // set flag to update values
        }
        else {
            arr8_stuff_rx[n8_rx] = buf_rx; // add byte to array
            n8_rx++; // increment number of bytes in packet
        }
    }

    MAP_Interrupt_enableMaster(); // enable all interrupts
}


void updateValues(void) { // TODO: change function name?
    // update values based on received UART data
    switch ((int)uart_rx[0] & 255) { // LSbyte of first float indicates message type
        volatile float *start; // pointer to start of data to copy

        case 0: { // update pressure setpoints
            start = uart_rx+1; // start after first float
            std::copy(start, start+NUM_VALVES, pres_des);
            break;
        }
        case 1: { // update pressure controller gains
            const int ctrl_idx = ((int)uart_rx[0]) >> 8 & 255; // 2nd LSbyte indicates index of pressure controller
            start = uart_rx+1;
            std::copy(start, start+4, ctrl_params[ctrl_idx]);
            break;
        }
        case 2: { // update length estimation coefficients TODO
            break;
        }
        case 3: { // update force estimation coefficients TODO
            break;
        }
    }
}