#include "header.h"

// initialize encoders based on current channel states, then enable encoder interrupts
void initEncoder(void) {
    int i; // encoder index
    int j; // channel index
    int encPinsOr; // bitwise OR of all encoder pins
    for (i = 0; i < 2; i++) { // loop over encoders
        uint8_t encChannelStates[2]; // create array for encoder channel states

        for (j = 0; j < 2; j++) { // loop over encoder channels A & B
            encChannelStates[j] = MAP_GPIO_getInputPinValue(GPIO_PORT_P3, encPins[i][j]); // get encoder channel state

            if (encChannelStates[j] == GPIO_INPUT_PIN_LOW) { // if channel is low
                MAP_GPIO_interruptEdgeSelect(GPIO_PORT_P3, encPins[i][j], GPIO_LOW_TO_HIGH_TRANSITION); // set low to high transition
            }
            else { // if channel is high
                MAP_GPIO_interruptEdgeSelect(GPIO_PORT_P3, encPins[i][j], GPIO_HIGH_TO_LOW_TRANSITION); // set high to low transition
            }
            encPinsOr |= encPins[i][j]; // add channel pin to bitwise OR
        }
        prevEncState[i] = (encChannelStates[0] << 1) | encChannelStates[1]; // set previous encoder state as current encoder state
        MAP_GPIO_interruptEdgeSelect(GPIO_PORT_P3, encPins[i][2], GPIO_LOW_TO_HIGH_TRANSITION); // set low to high transition for channel X
        encPinsOr |= encPins[i][2]; // add channel pin to bitwise OR
    }
    MAP_GPIO_clearInterruptFlag(GPIO_PORT_P3, encPinsOr); // clear all interrupt flags
    MAP_GPIO_enableInterrupt(GPIO_PORT_P3, encPinsOr); // enable GPIO interrupt
    MAP_Interrupt_enableInterrupt(INT_PORT3); // enable interrupt in NVIC
}


// read encoders: GPIO port 3 interrupt handler
void PORT3_IRQHandler(void) {
    MAP_Interrupt_disableMaster(); // disable interrupts
    uint_fast16_t status = MAP_GPIO_getEnabledInterruptStatus(GPIO_PORT_P3); // get port 3 interrupt status

    int encNum; // encoder number (0 for encoder 1, 1 for encoder 2)
    if (status <= (encPins[0][0] | encPins[0][1] | encPins[0][2])) { // if encoder 1 interrupt
        encNum = 0; // interrupt is for encoder 1
    }
    else {
        encNum = 1; // interrupt is for encoder 2
    }

    uint8_t encChannelStates[2] = {MAP_GPIO_getInputPinValue(GPIO_PORT_P3, encPins[encNum][0]), // get encoder channels A,B values
                                   MAP_GPIO_getInputPinValue(GPIO_PORT_P3, encPins[encNum][1])};
    uint8_t encState = (encChannelStates[0] << 1) | encChannelStates[1]; // combine channels into encoder state

    if (status & encPins[encNum][2]) { // if index interrupt
        encCounts[encNum] = 0; // reset encoder count
    }
    else { // if A,B interrupts
        uint8_t encStateCombined = (encState << 2) | prevEncState[encNum]; // combine current state and previous state
        int8_t countInc; // encoder counter increment
        switch (encStateCombined) {
            case 0: // do nothing
                countInc = 0;
                break;
            case 1: // count up
                countInc = 1;
                break;
            case 2: // count down
                countInc = -1;
                break;
            case 3: // do nothing - this is an error
                countInc = 0;
                break;
            case 4: // count down
                countInc = -1;
                break;
            case 5: // do nothing
                countInc = 0;
                break;
            case 6: // do nothing - this is an error
                countInc = 0;
                break;
            case 7: // count up
                countInc = 1;
                break;
            case 8: // count up
                countInc = 1;
                break;
            case 9: // do nothing - this is an error
                countInc = 0;
                break;
            case 10: // do nothing
                countInc = 0;
                break;
            case 11: // count down
                countInc = -1;
                break;
            case 12: // do nothing - this is an error
                countInc = 0;
                break;
            case 13: // count down
                countInc = -1;
                break;
            case 14: // count up
                countInc = 1;
                break;
            case 15: // do nothing
                countInc = 0;
                break;
        }
        encCounts[encNum] = encCounts[encNum] + countInc; // increment/decrement encoder count appropriately
    }

    int j; // channel index
    for (j = 0; j < 2; j++) { // loop over encoder channels A & B
       if (encChannelStates[j] == GPIO_INPUT_PIN_LOW) { // if channel is low
           MAP_GPIO_interruptEdgeSelect(GPIO_PORT_P3, encPins[encNum][j], GPIO_LOW_TO_HIGH_TRANSITION); // set low to high transition
       }
       else { // if channel is high
           MAP_GPIO_interruptEdgeSelect(GPIO_PORT_P3, encPins[encNum][j], GPIO_HIGH_TO_LOW_TRANSITION); // set high to low transition
       }
    }

    prevEncState[encNum] = encState; // set previous encoder state to current encoder state
    MAP_GPIO_clearInterruptFlag(GPIO_PORT_P3, status); // clear interrupt flag
    MAP_Interrupt_enableMaster(); // enable interrupts
}
