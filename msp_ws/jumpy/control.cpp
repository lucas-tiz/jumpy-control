#include "header.h"


// PID pressure control: maintain desired pressure setpoints
void controlUpdate(void) {
    float err;
    static float errInt[NUM_PRES_SENSOR] = {0};
    float u;
    float duty;

    // loop over pressure control inputs
    int i;
    for (i = 0; i < NUM_PRES_SENSOR-1; i++) {

        err = presFilt[i][0] - pres_des[i]; // (psi) calculate pressure error
        errInt[i] = errInt[i] + err; // (psi-time) integrate pressure error

        // reset integral windup
        if (errInt[i]*ctrl_params[i][1] > ctrl_params[i][3]) {
            errInt[i] = ctrl_params[i][3]/ctrl_params[i][1];
        }
        else if (errInt[i]*ctrl_params[i][1] < -ctrl_params[i][3]) {
            errInt[i] = -ctrl_params[i][3]/ctrl_params[i][1];
        }

        // calculate raw input
        u = (ctrl_params[i][2]*pres_des[i] + 10) - ctrl_params[i][0]*err - ctrl_params[i][1]*errInt[i] ; // input: (ff + deadzone offset) + prop + int
        duty = u;

        // impose saturation limits
        if (duty > 100.0) {
            duty = 100.0;
        }
        else if (duty < 10.0) {
            duty = 0.0;
        }

        // if desired pressure is zero, just set duty cycle to zero
        if (pres_des[i] == 0) {
            duty = 0.0;
            errInt[i] = 0; // reset integral
        }

        // update PWM timer duty cycle
        if (i == 0) {
            uart_tx[3] = err;
            uart_tx[4] = duty;

            TIMER_A0->CCR[1] = (int)((duty/100.0)*PWM_PERIOD);
            TIMER_A0->CCR[2] = (int)((duty/100.0)*PWM_PERIOD);
            TIMER_A0->CCR[3] = (int)((duty/100.0)*PWM_PERIOD);
        }
//        TIMER_A0->CCR[i+1] = (int)((duty/100.0)*PWM_PERIOD);
    }




//    // pump control
//    if (presFilt[4][0] > 14) { // 10 for free actuation
//        MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P6, GPIO_PIN6 | GPIO_PIN7);// disable pumps
//    }
//    else if (presFilt[4][0] < 12) { // 8 for free actuation
//        MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P6, GPIO_PIN6 | GPIO_PIN7); // enable pumps
//    }
}


// control update timer interrupt routine
extern "C" void TA2_0_IRQHandler(void) {
    MAP_Interrupt_disableMaster(); // disable interrupts
    controlFlag = 1; // set flag to get update system control
    MAP_Timer_A_clearCaptureCompareInterrupt(TIMER_A2_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_0); // clear interrupt flag
    MAP_Interrupt_enableMaster(); // enable interrupts
}

