
#include "MS5611.h"

const uint16_t FREQUENCY = 100; //Hz del bucle principal
const uint16_t SAMPLETIME = 10000; //1000000/FRECUENCIA µs

int PIDaltimetro = 0;

void setup() {
    Serial.begin(115200);
    
    initMS5611();
    delay (10); //delay de 10us que simulan el buclue de 250Hz
}

void loop() {
    calculateAltitude();
    delay(10); //delay de 10us que simulan el buclue de 250Hz
}

/*
MS.ino       18
MS5611.h     47
MS5611.cpp  303

0***
0******
0********
*/
