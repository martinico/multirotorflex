
/*
https://github.com/lobodol/ESC-calibration
@author lobodol <grobodol@gmail.com>
*/

#include <ESP32Servo.h> //¡¡¡¡ ojo #include <Servo.h> para maquinas Arduino-UNO !!!

#define MIN_PULSE_LENGTH 1000 // Minimum pulse length in µs
#define MAX_PULSE_LENGTH 2000 // Maximum pulse length in µs

Servo ESC; 

char data;

void setup() {
    Serial.begin(115200);

    ESC.setPeriodHertz(250); //quitar para Arduino-UNO
    ESC.attach(33, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH); //27=PROA 25=POPA 26=ESTRIBOR  33=BABOR
    
    displayInstructions();
}

void loop () {
    if (Serial.available()) {
        data = Serial.read();

        switch (data) {
            case 48 : Serial.println("Sending minimum throttle"); //0
                      ESC.writeMicroseconds(MIN_PULSE_LENGTH);
                      break;
            case 49 : Serial.println("Sending maximum throttle"); //1
                      ESC.writeMicroseconds(MAX_PULSE_LENGTH);
                      break;
            case 50 : Serial.print("Running test in "); //2
                      delay(1000);
                      Serial.print("3 ");
                      delay(1000);
                      Serial.print("2 ");
                      delay(1000);
                      Serial.println("1 ... ");
                      delay(1000);
                      test();
                      break;
        }
    }
}

void displayInstructions() {  
  Serial.println(F("INSTRUCTIONS"));
  Serial.println(F("------------"));
  Serial.println(F(""));
  Serial.println(F("A. After having uploaded your sketch and having ESCs NOT powered up yet, open the terminal and TYPE 1."));
  Serial.println(F("   This will send MAX throttle to ESC in order to make them enter in programming mode."));
  Serial.println(F(""));
  Serial.println(F("B. Power up your ESC. You must hear \"beep1 beep2 beep3\" short tones meaning the power supply is OK."));
  Serial.println(F("   After 2 seconds, \"beep beep\" large tone emits, meaning the throttle highest point has been correctly confirmed."));
  Serial.println(F(""));
  Serial.println(F("C. Then, TYPE 0 to send MIN throttle. This will set the lowest throttle level for the ESC."));
  Serial.println(F("   Several \"beep\" tones emits, wich means the quantity of the lithium battery cells (3 beeps for a 3 cells LiPo)."));
  Serial.println(F("   A long beep tone emits meaning the throttle lowest point has been correctly confirmed. "));
  Serial.println(F("   OJO:la secuencia \"..-\" (punto,punto,raya) o incluso una escala de notas :-))) ouede confirmar el fin de la calibracion."));
  Serial.println(F("   Your ESC's are now well calibrated and ready for test."));
  Serial.println(F(""));
  Serial.println(F("D. Plug OFF and ON your ESC to proceed to testing it."));
  Serial.println(F("   TYPE 2 to launch test function."));
  Serial.println(F("   You must see your motors starting to run with increasing speed, then stop when maximum speed is reached."));
  Serial.println(F(""));
                
    Serial.println("READY - PLEASE SEND INSTRUCTIONS AS FOLLOWING :");
    Serial.println("\t0 : Send min throttle");
    Serial.println("\t1 : Send max throttle");
    Serial.println("\t2 : Run test function\n");
}

void test() {
    for (int i = MIN_PULSE_LENGTH; i <= MAX_PULSE_LENGTH; i += 10) {
        Serial.print("Pulse length = ");
        Serial.println(i);        
        ESC.writeMicroseconds(i);
        delay(1000);
    }
    Serial.println("STOP");
    ESC.writeMicroseconds(MIN_PULSE_LENGTH);
}
