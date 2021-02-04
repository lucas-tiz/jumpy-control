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
#include "device_conversion.hpp"

#include <algorithm> //TODO: needed?
#include <math.h>



////////////////////////////////////////////////////////////////////////////////
/* Macros */
#define NUM_PRES_SENSOR 5  // 5 max, based on number of provided PWM signals

#define LPF_ORDER 20 // pressure sensor LPF

#define MAX_TRAJ_DATA 2000 /// maximum length of trajectory data to store


////////////////////////////////////////////////////////////////////////////////
/* Global variables */
bool g_flag_start_seq = 0;
bool g_flag_valve_seq = 0;
bool g_flag_dump = 0;

volatile int g_flag_sense = 0;
volatile bool g_flag_control = 0;
volatile bool g_flag_transmit = 0;

volatile float g_t_valve_seq = 0;

int g_len_valve_seq;
float valve_seq[MAX_VALVE_SEQ][NUM_VALVES+1];


////////////////////////////////////////////////////////////////////////////////
/* Prototypes */
extern "C" void TA1_0_IRQHandler(void);
extern "C" void TA2_0_IRQHandler(void);
extern "C" void TA3_0_IRQHandler(void);
extern "C" void SysTick_Handler(void);

void receiveData(void);
void sensorUpdate(const int * idx_adc_pres, float pres[][LPF_ORDER+1],
    const float * lpf_coeffs);
void controlUpdate(float pres[][LPF_ORDER+1]);


////////////////////////////////////////////////////////////////////////////////
/* Configure MSP */
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



    ////////////////////////////////////////////////////////////////////////////////
    /* Local variables */
    const int idx_adc_pres[NUM_PRES_SENSOR] = {0,2,3,4,5}; // ADC channel corresponding to pressure

    const float lpf_coeffs[LPF_ORDER+1] =
        {0.00007967,0.00097911,0.00371099,0.00985210,0.02087523, // FIR low-pass filter coefficients TODO: adjust
         0.03747917,0.05886406,0.08241999,0.10409454,0.11943855,
         0.12498883,0.11943855,0.10409454,0.08241999,0.05886406,
         0.03747917,0.02087523,0.00985210,0.00371099,0.00097911,
         0.00007967};


    float pres[NUM_PRES_SENSOR][LPF_ORDER+1]; // (psi) current and previous pressures
//    float presFilt[NUM_PRES_SENSOR][2]; // (psi) current and one previous time-step of filtered pressures

    float uart_tx[63] = {0.0f};
    float data_traj[MAX_TRAJ_DATA][6] = {0.0f};

    int idx_seq = 0; // valve sequence index
    int idx_sense = 0; // sensor read index
    int n_sense = 0; // number of sensor reads


    ////////////////////////////////////////////////////////////////////////////
    /* Run */
    while(1) {
        // start valve time sequence
        if (1 == g_flag_start_seq) {
            MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN3); // turn on external LED
            g_t_valve_seq = 0;
            idx_seq = 0;
            idx_sense = 0;

            g_flag_start_seq = 0;
            g_flag_valve_seq = 1;
        }


        // run valve time sequence
        if (1 == g_flag_valve_seq) {
            if (g_t_valve_seq >= valve_seq[idx_seq][0]) { // if at next time in sequence
                std::copy(valve_seq[idx_seq]+1, valve_seq[idx_seq]+1+NUM_VALVES, pres_des); // update pressure setpoints
                controlUpdate(pres);
                idx_seq++;
            }
            if (idx_seq == g_len_valve_seq) { // if at end of sequence
                MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN3); // turn off external LED
                n_sense = idx_sense;
                g_flag_valve_seq = 0;
            }
        }


        // dump trajectory data via UART
        if (g_flag_dump) {
            MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN0); // toggle red LED

            for (int i = 0; i < n_sense; i++) {
                sendData(data_traj[i], 6, 2); // send row of data array
            }
            sendData(data_traj[0], 1, 3); // signal dump is complete
            g_flag_dump = 0;

            MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN0); // toggle red LED

        }


        // sense, control, receive, transmit
        if (g_flag_sense) {

            if (1 == g_flag_sense) {
                MAP_ADC14_toggleConversionTrigger();
                g_flag_sense = 2;
            }

            if (!MAP_ADC14_isBusy()) {
//                MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN5); // DEBUG
                sensorUpdate(idx_adc_pres, pres, lpf_coeffs);
//                MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN5); // DEBUG

                if (g_flag_valve_seq) {
//                    float s = 0.1f;
                    data_traj[idx_sense][0] = g_t_valve_seq;
                    data_traj[idx_sense][1] = pres[0][0]; //sin(g_t_valve_seq) + (float)rand()/RAND_MAX*s;
                    data_traj[idx_sense][2] = pres[1][0]; //sin(g_t_valve_seq-0.1f) + (float)rand()/RAND_MAX*s;
                    data_traj[idx_sense][3] = pres[2][0]; //sin(g_t_valve_seq-0.2f) + (float)rand()/RAND_MAX*s;
                    data_traj[idx_sense][4] = pres[3][0]; //sin(g_t_valve_seq-0.3f) + (float)rand()/RAND_MAX*s;
                    data_traj[idx_sense][5] = pres[4][0]; //sin(g_t_valve_seq-0.4f) + (float)rand()/RAND_MAX*s;
                    idx_sense += 1;
                }
                if (!g_flag_valve_seq && !g_flag_dump) {
                    uart_tx[0] = pres[0][0]; //sin(g_t_valve_seq);
                    uart_tx[1] = pres[1][0]; //sin(g_t_valve_seq-0.1f);
                    uart_tx[2] = pres[2][0]; //sin(g_t_valve_seq-0.2f);
                    uart_tx[3] = pres[3][0]; //sin(g_t_valve_seq-0.3f);
                    uart_tx[4] = pres[4][0]; //sin(g_t_valve_seq-0.4f);
                }
                g_flag_sense = 0;
            }
        }
        if (g_flag_control) {
//            MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN7); // DEBUG
            controlUpdate(pres);
//            MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN7); // DEBUG
            g_flag_control = 0; // clear flag
        }
        if (flag_receive) {
            receiveData();
            flag_receive = 0; // clear flag
        }
        if (g_flag_transmit && !g_flag_valve_seq && !g_flag_dump) { // if not running traj
            sendData(uart_tx, 5, 1);
            g_flag_transmit = 0; // clear flag
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


////////////////////////////////////////////////////////////////////////////////
/* Interrupts */
extern "C" void TA1_0_IRQHandler(void) {
    /* Timer 1 interrupt: set sense flag */
    g_flag_sense = 1;
    MAP_Timer_A_clearCaptureCompareInterrupt(TIMER_A1_BASE,
        TIMER_A_CAPTURECOMPARE_REGISTER_0);
}


extern "C" void TA2_0_IRQHandler(void) {
    /* Timer 2 interrupt: set control flag */
    g_flag_control = 1; // set flag to get update system control
    MAP_Timer_A_clearCaptureCompareInterrupt(TIMER_A2_BASE,
        TIMER_A_CAPTURECOMPARE_REGISTER_0); // clear interrupt flag
}


extern "C" void TA3_0_IRQHandler(void) {
    /* Timer 3 interrupt: set data transmit flag */
    g_flag_transmit = 1; // set flag to get sensor values
    MAP_Timer_A_clearCaptureCompareInterrupt(TIMER_A3_BASE,
        TIMER_A_CAPTURECOMPARE_REGISTER_0); // clear interrupt flag
}


extern "C" void SysTick_Handler(void) {
    /* SysTick timer interrupt: increment time */
    g_t_valve_seq += 0.0001f;
}


////////////////////////////////////////////////////////////////////////////////
/* Functions */
void receiveData(void) {
    /* Do something with UART packet based on packet type */
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
            int idx_seq = ((int)uart_rx[0]) >> 8 & 255; // 2nd LSbyte indicates index pressure sequence
            g_len_valve_seq = idx_seq+1; // update sequence length
            start = uart_rx+1;
            std::copy(start, start+5, valve_seq[idx_seq]); // copy time + 4 pressure values into sequence array
            break;
        }
        case 4: { // start valve time sequence
            g_flag_start_seq = 1;
            break;
        }
        case 5: { // data dump via UART
            g_flag_dump = 1;
            break;
        }
    }
}


// convert encoder counts to angles, filter, and save
void sensorUpdate(const int * idx_adc_pres, float pres[][LPF_ORDER+1],
    const float * lpf_coeffs) {
    /* Read sensors */

//    // shift angle histories
//    theta1[1] = theta1[0];
//    theta2[1] = theta2[0];
//
//    // convert encoder counts to angles and save as current angles (add negative to reverse encoder direction)
//    theta1[0] = -((float)(encCounts[0])*DEG_PER_COUNT + THETA_1_OFFSET); // convert base wheel counts to angle
//    theta2[0] = -((float)(encCounts[1])*DEG_PER_COUNT + THETA_2_OFFSET); // convert wheel offset counts to angle

    // get ADC conversions
    uint16_t adc_conv[9]; // initialize array for ADC results
    for (int i = 0; i < 9; i++) {
        adc_conv[i] = ADC14->MEM[i];
    }

    // update pressures & perform pressure calculations (pressures correspond to A0 - A4)
    int idx_adc;
    for (int i = 0; i < NUM_PRES_SENSOR; i++) { // loop over pressure sensors

//        // update pressure history
//        for (int j = 0; j < LPF_ORDER; j++) { // loop over pressure history
//          pres[i][LPF_ORDER-j] = pres[i][LPF_ORDER-(j+1)]; // shift each value right (20 = 19, 19 = 18,...)
//        }
        idx_adc = idx_adc_pres[i]; // index corresponding to ADC channel used for pressure measurement
        if (i == 0) { // first (tank) transducer is old ASDX type
//            pres[i][0] = convert.intToFloat(adc_conv[idx_adc]*2, PRES_ASDXAVX100PGAA5); // convert and update pressure
            pres[i][0] = 0.02014652014f*(float)adc_conv[idx_adc]*2 - 12.5f; //NOTE: x2 for voltage divider
        }
        else {
//            pres[i][0] = convert.intToFloat(adc_conv[idx_adc], PRES_SSCDRRN100PGAB5); // convert and update pressure
            pres[i][0] = 0.0179080179f*(float)adc_conv[idx_adc] - 5.55555555556f;
        }
//        // calculate filtered pressures
//        presFilt[i][1] = presFilt[i][0]; // shift filtered pressure histories
//        presFilt[i][0] = 0; // reset current value to zero
//        for (int j = 0; j <= LPF_ORDER; j++) { // loop over pressure history TODO: coeffs or pres loop dir needs to be switched
//            presFilt[i][0] = presFilt[i][0] + lpf_coeffs[j]*pres[i][j]; // calculate filtered value
//        }
    }
}


void controlUpdate(float pres[][LPF_ORDER+1]) {
    /* 3-way inflate/seal/vent valve control */

    static float pres_deadband[4] = {15,15,15,15};
    static float pres_undershoot[4] = {0,0,0,0};
    int idx_valve; // index of actuator
    int idx_trans; // index of pressure transducer
    for (idx_valve = 0; idx_valve < NUM_VALVES; idx_valve++) { // loop over actuators
        idx_trans = idx_valve + 1;

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
        else if (pres[idx_trans][0] < (pres_des[idx_valve] - pres_undershoot[idx_valve])) {
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
}
