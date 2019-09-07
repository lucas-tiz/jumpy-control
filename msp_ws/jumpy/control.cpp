#include "header.h"



void controlUpdate(void) {
    // 3-way inflate/seal/vent valve control
    int idx_valve; // index of actuator
    int idx_trans; // index of pressure transducer
    for (idx_valve = 0; idx_valve < NUM_VALVES; idx_valve++) { // loop over actuators
        idx_trans = idx_valve + 1;

        // pressure on/off control
        //TODO: set pres_deadband
//        if (pres_des[idx_valve] == 0) { // vent
//            MAP_GPIO_setOutputHighOnPin(valves[idx_valve].port, valves[idx_valve].pin_vent);
//            MAP_GPIO_setOutputLowOnPin(valves[idx_valve].port, valves[idx_valve].pin_inflate);
//        }
//        else if (pres[idx_trans][0] >= pres_des[idx_valve]) { // seal
//            MAP_GPIO_setOutputLowOnPin(valves[idx_valve].port, valves[idx_valve].pin_vent);
//            MAP_GPIO_setOutputLowOnPin(valves[idx_valve].port, valves[idx_valve].pin_inflate);
//        }
//        else if (pres[idx_trans][0] < pres_des[idx_valve] - pres_deadband[idx_valve]) { // inflate
//            MAP_GPIO_setOutputLowOnPin(valves[idx_valve].port, valves[idx_valve].pin_vent);
//            MAP_GPIO_setOutputHighOnPin(valves[idx_valve].port, valves[idx_valve].pin_inflate);
//        }

        // manual on/off control
        if (pres_des[idx_valve] == 0) { // vent
            MAP_GPIO_setOutputHighOnPin(valves[idx_valve].port, valves[idx_valve].pin_vent);
            MAP_GPIO_setOutputLowOnPin(valves[idx_valve].port, valves[idx_valve].pin_inflate);
        }
        else if (pres_des[idx_valve] > 0) { // inflate
            MAP_GPIO_setOutputLowOnPin(valves[idx_valve].port, valves[idx_valve].pin_vent);
            MAP_GPIO_setOutputHighOnPin(valves[idx_valve].port, valves[idx_valve].pin_inflate);
        }
        else { // (negative number) // seal
            MAP_GPIO_setOutputLowOnPin(valves[idx_valve].port, valves[idx_valve].pin_vent);
            MAP_GPIO_setOutputLowOnPin(valves[idx_valve].port, valves[idx_valve].pin_inflate);
        }
    }
}


// control update timer interrupt routine
extern "C" void TA2_0_IRQHandler(void) {
    MAP_Interrupt_disableMaster(); // disable interrupts
    controlFlag = 1; // set flag to get update system control
    MAP_Timer_A_clearCaptureCompareInterrupt(TIMER_A2_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_0); // clear interrupt flag
    MAP_Interrupt_enableMaster(); // enable interrupts
}

