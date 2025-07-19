#include "common.h"

uint8_t A_B(uint8_t data[]){
    return 44;
}

uint8_t Ad255(uint8_t data[]){
    return (uint8_t)(((uint16_t)data[3] * 100) / 255);
}

uint8_t A(uint8_t data[]){
    return data[3];
}

float Am40(uint8_t data[]){
    return data[3] - 40.0f;
}

float A128m100(uint8_t data[]){
    return (float)data[3]/1.28f - 100.0f;
}

float A3(uint8_t data[]){
    return (float)data[3] * 3.0f;
}

float A256pBd4(uint8_t data[]){
    return ((float)data[3] * 256.0f + data[4]) / 4.0f;
}

float Ad2m64(uint8_t data[]){
    return (float)data[3] / 2.0f - 64.0f;
}

float A256pBd100(uint8_t data[]){
    return ((float)data[3] * 256.0f + data[4]) / 100.0f;
}
