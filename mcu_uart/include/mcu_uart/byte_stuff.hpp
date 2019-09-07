// Stuff and de-stuff array of bytes
/*
IEEE-754 floating point converter: https://www.h-schmidt.net/FloatConverter/IEEE754.html
*/

#ifndef BYTE_STUFF_HPP_
#define BYTE_STUFF_HPP_


#include <stdint.h>
#include <sstream>
#include <cstring>



void castFloatToInts(float *f, uint8_t *int8_arr) {
    std::memcpy(int8_arr, f, 4);
}


void castIntsToFloat(uint8_t *int8_arr, float *f) {
    std::memcpy(f, int8_arr, 4);
}



// separate floats into uint8_t bytes
void SeparateArr(float *arr_float, int len_float, uint8_t *arr8_unstuff) {
    for (int i = 0; i < len_float; i++) {
        castFloatToInts(&arr_float[i], &arr8_unstuff[i*4]);
    }
}


// stuff byte array
void StuffArr(uint8_t *arr8_unstuff, int len8_unstuff, uint8_t *arr8_stuff) {
    int iz = 0; // zero index (index of offset byte to be modified)
    int oz = 0; // zero offset (offset to next zero byte)
    for (int id = 0; id < len8_unstuff; id++) { // data index
        oz++; // increment zero offset
    
        if (arr8_unstuff[id] == 0x00) {
            arr8_stuff[iz] = oz; // set offset byte to zero offset
            iz = id+1; // set zero index
            oz = 0; // reset zero offset
        }
        else {
            arr8_stuff[id+1] = arr8_unstuff[id]; // add data to stuffed array
        }
    }
    arr8_stuff[iz] = ++oz; // set last offset byte (to end of packet)
    arr8_stuff[len8_unstuff+1] = 0; // set last element to zero
}
    
    
// un-stuff byte array
void UnstuffArr(uint8_t *arr8_stuff, int len8_stuff, uint8_t *arr8_unstuff) {
    int iz =  arr8_stuff[0]; // zero index
    for (int ids = 1; ids < (len8_stuff-1); ids++) { // loop over stuffed data
        
        if (ids == iz) { // if stuffed data arr index equals zero index
            arr8_unstuff[ids-1] = 0; // set un-stuffed data to zero
            iz = ids + arr8_stuff[ids]; // set next zero index
        }
        else {
            arr8_unstuff[ids-1] = arr8_stuff[ids]; // copy stuffed data directly
        }
    }    
}


// concatenate uint8_t bytes into floats
void ConcatArr(uint8_t *arr8_unstuff, int len8_unstuff, float *arr_float) {
    for (int i = 0; i < len8_unstuff/4; i++) {
        castIntsToFloat(&arr8_unstuff[i*4], &arr_float[i]);
    }
}


// concatenate uint8_t array bytes into uint16_ts
uint16_t ConcatData(uint8_t first_byte, uint8_t second_byte) {
    uint16_t data_concat = (uint16_t)(first_byte << 8) | (uint16_t)second_byte; // concatenate bytes
    return data_concat;
}


#endif // BYTE_STUFF_HPP_
