#include "header.h"
#include <vector>
#include <array>
#include <algorithm>
#include <iterator>
#include "byte_stuff.hpp"


// send data over UART
void sendData(float * arr_float, int n_float, uint8_t msg_type) {
    // set up data structures
    int n8 = n_float*4; // number of data bytes to transmit
    uint8_t arr8_unstuff[252]; // array for unstuffed data
    uint8_t arr8_stuff[256]; // array for stuffed data

    // separate and stuff data
    SeparateArr(arr_float, n_float, arr8_unstuff);  // separate floats into bytes
    StuffArr(arr8_unstuff, n8, arr8_stuff); // create stuffed byte packet

    // send message type
    MAP_UART_transmitData(EUSCI_A0_BASE, msg_type); // send byte
    while (MAP_UART_getInterruptStatus(EUSCI_A0_BASE, EUSCI_A_UART_TRANSMIT_INTERRUPT_FLAG)
            != EUSCI_A_UART_TRANSMIT_INTERRUPT_FLAG); // wait for transmission completion

    // send data bytes
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

