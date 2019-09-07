#include "header.h"

// convert comma-separated value string to floats, write values to array
void csvStringRead(int nVals, volatile uint8_t * strRead, volatile float * arrWrite) {
    char val[10]; // initialize value string
    int iStr = 0;  // full string index, initialize to 0
    int iArr;      // float array index
    int iVal = 0;  // number string index, initialize to 0

    for (iArr = 0; iArr < nVals; iArr++) { // loop over expected number of floats
       while (strRead[iStr] != ',') { // loop over string until comma reached
           val[iVal] = strRead[iStr]; // add full string char to value string
           iVal++; // increment value string index
           iStr++; // increment full string index
       }

       // when comma reached
       val[iVal] = '\0'; // add end of string char
       arrWrite[iArr] = atof(val); // convert value string to float, add to value array

       iVal = 0; // reset value string index
       iStr++; // increment full string index to next char after comma
    }
}

// software delay
void delay(int d)
{
    int i;
    for (i = 0; i < d; i++);
}
