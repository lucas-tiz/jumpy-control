#include "header.h"
#include <vector>
#include <array>
#include <algorithm>
#include <iterator>
#include "byte_stuff.hpp"


// send data over UART
void sendData(float * arr_float, int n_float) {
    // set up data structures
    int n8 = n_float*4; // number of data bytes to transmit
    uint8_t arr8_unstuff[252]; // array for unstuffed data
    uint8_t arr8_stuff[256]; // array for stuffed data

    // separate, stuff, and send data
    SeparateArr(arr_float, n_float, arr8_unstuff);  // separate floats into bytes
    StuffArr(arr8_unstuff, n8, arr8_stuff); // create stuffed byte packet
    for (int i = 0; i < (n8+2); i++) { // loop over data bytes + 2 stuff bytes
        MAP_UART_transmitData(EUSCI_A0_BASE, arr8_stuff[i]); // transmit byte
        while (MAP_UART_getInterruptStatus(EUSCI_A0_BASE, EUSCI_A_UART_TRANSMIT_INTERRUPT_FLAG)
                != EUSCI_A_UART_TRANSMIT_INTERRUPT_FLAG); // wait for transmission completion
    }
}


// receive data over UART: eUSCI A module interrupt routine
void EUSCIA0_IRQHandler(void) {
    MAP_Interrupt_disableMaster(); // disable all interrupts TODO: necessary?

    static uint8_t arr8_stuff[256]; // array for stuffed data
    static uint8_t arr8_unstuff[252]; // array for unstuffed data
    static float arr_float_cat[63]; // array for concatenated data
    static uint8_t buf; // byte receive buffer
    static int n8 = 0; // number of bytes received in packet

    if (MAP_UART_getInterruptStatus(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG)
            == EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG) { // if receive interrupt flag

        buf = MAP_UART_receiveData(EUSCI_A0_BASE); // access received data by reading RXBUF register; flag automatically cleared
        if (buf == 0) { // if 0 received
            arr8_stuff[n8] = buf; // add byte to array
            n8++; // increment number of bytes in packet

            UnstuffArr(arr8_stuff, n8, arr8_unstuff); // unstuff data in packet (auto clears unstuff vector)
            ConcatArr(arr8_unstuff, n8-2, arr_float_cat); // concatenate bytes into uint16_ts (auto clears cat vector)
            int n_float = (n8-2)/4; // number of floats received
            std::copy(arr_float_cat, arr_float_cat + n_float, uart_rx); // add received data into global array
            n8 = 0; // reset packet length
            flag_receive = 1; // set flag to update values
        }
        else {
            arr8_stuff[n8] = buf; // add byte to array
            n8++; // increment number of bytes in packet
        }
    }

    MAP_Interrupt_enableMaster(); // enable all interrupts
}


void receiveData(void) {
    // do something with data received via UART
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
        case 2: { // toggle external LED
            switch ((int)uart_rx[1] & 255) {
                case 0: {
                    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN3);
                    break;
                }
                case 1: {
                    MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN3);
                    break;
                }
            }
            break;
        }
        case 3: { // update valve time sequence array
            const int idx_seq = ((int)uart_rx[0]) >> 8 & 255; // 2nd LSbyte indicates index pressure sequence
            len_valve_seq = idx_seq; // update sequence length
            start = uart_rx+1;
            std::copy(start, start+5, valve_seq[idx_seq]); // copy time + 4 pressure values into sequence array
            break;
        }
        case 4: { // start valve time sequence
            t_valve_seq = 0; // reset valve sequence time
            flag_valve_seq = 1; // raise flag
            break;
        }
        case 5: { // send data back via UART
            flag_dump = 1; // raise flag
            break;
        }

    }
}
