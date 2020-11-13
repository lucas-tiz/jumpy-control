#include "header.h"


void controlUpdate(void) {
    // 3-way inflate/seal/vent valve control
//    MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN5); //DEBUG

    static float pres_deadband[4] = {15,15,15,15};
    static float pres_undershoot[4] = {0,0,0,0};
    int idx_valve; // index of actuator
    int idx_trans; // index of pressure transducer
    for (idx_valve = 0; idx_valve < NUM_VALVES; idx_valve++) { // loop over actuators
        idx_trans = idx_valve + 1;

//        pres[idx_trans][0] = 0; //DEBUG

        // seal (manual)
        if (pres_des[idx_valve] < 0) {
            MAP_GPIO_setOutputLowOnPin(valves[idx_valve].port, valves[idx_valve].pin_vent);
            MAP_GPIO_setOutputLowOnPin(valves[idx_valve].port, valves[idx_valve].pin_inflate);
        }

        // vent (manual)
        else if (pres_des[idx_valve] == 0) {
            MAP_GPIO_setOutputHighOnPin(valves[idx_valve].port, valves[idx_valve].pin_vent);
            MAP_GPIO_setOutputLowOnPin(valves[idx_valve].port, valves[idx_valve].pin_inflate);
        }

        // inflate: pressure < setpoint - undershoot
        else if (pres[idx_trans][0] < (pres_des[idx_valve] - pres_undershoot[idx_valve])) { // inflate
            MAP_GPIO_setOutputLowOnPin(valves[idx_valve].port, valves[idx_valve].pin_vent);
            MAP_GPIO_setOutputHighOnPin(valves[idx_valve].port, valves[idx_valve].pin_inflate);
        }

        // seal (feedback control): pressure <= setpoint + deadband
        else if (pres[idx_trans][0] <= (pres_des[idx_valve] + pres_deadband[idx_valve])) {
            MAP_GPIO_setOutputLowOnPin(valves[idx_valve].port, valves[idx_valve].pin_vent);
            MAP_GPIO_setOutputLowOnPin(valves[idx_valve].port, valves[idx_valve].pin_inflate);
        }

        // vent: pressure > setpoint + deadband
        else {
            MAP_GPIO_setOutputHighOnPin(valves[idx_valve].port, valves[idx_valve].pin_vent);
            MAP_GPIO_setOutputLowOnPin(valves[idx_valve].port, valves[idx_valve].pin_inflate);
        }


    }
//    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN5); //DEBUG
}


// control update timer interrupt routine
extern "C" void TA2_0_IRQHandler(void) {
    flag_control = 1; // set flag to get update system control
    MAP_Timer_A_clearCaptureCompareInterrupt(TIMER_A2_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_0); // clear interrupt flag
}

