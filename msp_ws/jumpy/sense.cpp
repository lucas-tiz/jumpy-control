#include "header.h"
#include "device_conversion.hpp"
#include <algorithm>

// convert encoder counts to angles, filter, and save
void sensorUpdate(void) {
    static Converter convert(ADC_RES14, 5.0, 3.3); // measurement converter

    // shift angle histories
    theta1[1] = theta1[0];
    theta2[1] = theta2[0];

    // convert encoder counts to angles and save as current angles (add negative to reverse encoder direction)
    theta1[0] = -((float)(encCounts[0])*DEG_PER_COUNT + THETA_1_OFFSET); // convert base wheel counts to angle
    theta2[0] = -((float)(encCounts[1])*DEG_PER_COUNT + THETA_2_OFFSET); // convert wheel offset counts to angle

    // get ADC conversions
    MAP_ADC14_toggleConversionTrigger(); // initiate a single conversion (trigger)
    while(MAP_ADC14_isBusy()); // wait until ADC conversion complete
    uint16_t adc_conv[14]; // initialize array for ADC results
    for (int i = 0; i < 14; i++) {
        adc_conv[i] = ADC14->MEM[i];
    }

    // update pressures & perform pressure calculations (pressures correspond to A0 - A4)
    int idx_adc;
    for (int i = 0; i < NUM_PRES_SENSOR; i++) { // loop over pressure sensors

        // update pressure history
        for (int j = 0; j < LPF_ORDER; j++) { // loop over pressure history
          pres[i][LPF_ORDER-j] = pres[i][LPF_ORDER-(j+1)]; // shift each value right (20 = 19, 19 = 18,...)
        }
        idx_adc = presAdc[i]; // index corresponding to ADC channel used for pressure measurement
        pres[i][0] = convert.intToFloat(adc_conv[idx_adc], PRES_SSCDRRN100PGAB5); // convert and update pressure

        // calculate filtered pressures
        presFilt[i][1] = presFilt[i][0]; // shift filtered pressure histories
        presFilt[i][0] = 0; // reset current value to zero
        for (int j = 0; j <= LPF_ORDER; j++) { // loop over pressure history TODO: coeffs or pres loop dir needs to be switched
            presFilt[i][0] = presFilt[i][0] + lpfCoeffs[j]*pres[i][j]; // calculate filtered value
        }
    }

    // update FSR
    float fsr_volt = convert.intToFloat(adc_conv[6], ANALOG); // convert


//    // update light sensors (light sensors correspond to A5 - A12)
//    for (int i = 0; i < NUM_LIGHT_SENSOR; i++) { // loop over light sensors
//        idx_adc = lightAdc[i]; // index corresponding to ADC channel used for light measurement
//        light[i] = adc_conv[idx_adc]*(3.3/16383); // update light sensor value
//    }

    // select data to transmit (put into uart_tx array) TODO: put this somewhere else when no longer directly using 'adc_conv'
    uart_tx[0] = presFilt[0][0];
    uart_tx[1] = presFilt[1][0];
    uart_tx[2] = fsr_volt;

//    std::copy(ctrl_params[0], ctrl_params[0]+4, uart_tx + 5); // @suppress("Function cannot be resolved")
//    std::copy(ctrl_params[1], ctrl_params[1]+4, uart_tx + 9);
//    std::copy(pres_des, pres_des+5, uart_tx+13);

}


// sensor update timer interrupt routine
extern "C" void TA1_0_IRQHandler(void) {
    MAP_Interrupt_disableMaster(); // disable interrupts
    sensorFlag = 1; // set flag to get sensor values
    MAP_Timer_A_clearCaptureCompareInterrupt(TIMER_A1_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_0); // clear interrupt flag
    MAP_Interrupt_enableMaster(); // enable interrupts
}

