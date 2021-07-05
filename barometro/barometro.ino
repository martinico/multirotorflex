
#include "Wire.h"

#define MS5611_ADDRESS 0x77   //I2C address of the MS5611

#define SDA1 4
#define SCL1 5

const float METROS_A_MILIBARES = 0.117; //delta(1 metro) => delta(0.117 milibar)

float MPF = 0.85; //BAND-PASS FILTER

//Pressure & Temperature variables.
//---------------------------------
uint8_t barometer_counter, temperature_counter = 0;
uint16_t C[7]; //obviamos el "0" 
/*
C1 Pressure sensitivity | SENST1
C2 Pressure offset | OFFT1
C3 Temperature coefficient of pressure sensitivity | TCS
C4 Temperature coefficient of pressure offset | TCO
C5 Reference temperature | TREF
C6 Temperature coefficient of the temperature | TEMPSENS
*/
float actual_pressure;
int64_t OFF, OFF_C2, SENS, SENS_C1, P;
int32_t dT, dT_C5;

const uint8_t TEMPERATURE_ROTATING_MEMORY = 5;
uint32_t raw_temperature_rotating_memory[TEMPERATURE_ROTATING_MEMORY+1]; //obviamos el "0" 
uint8_t average_temperature_mem_location;

const uint8_t PRESSURE_ROTATING_MEMORY = 20;
int32_t pressure_rotating_mem[PRESSURE_ROTATING_MEMORY +1]; //obviamos el "0" 
uint8_t pressure_rotating_mem_location;

uint32_t raw_pressure, raw_temperature, temp, raw_average_temperature_total;
float actual_pressure_slow, actual_pressure_fast, actual_pressure_diff;
int32_t pressure_total_avarage;
float pressure_rotating_mem_actual;

//Altitude PID variables.
//-----------------------
float ground_pressure, altitude_hold_pressure;
float pid_i_mem_altitude, pid_altitude_setpoint, pid_altitude_input, pid_output_altitude, pid_last_altitude_d_error;
uint8_t parachute_rotating_mem_location;
int32_t parachute_buffer[35], parachute_throttle;
float pressure_parachute_previous;



void setup() {
  Wire1.begin(SDA1, SCL1);                  // activa el bus I2C secundario
  Wire1.setClock(400000);                   // reloj = 100kHz (comenta la sentenca si encuentras dificultades)
  Wire1.beginTransmission(MS5611_ADDRESS);
  uint8_t error = Wire1.endTransmission();
  while (error != 0) {
    Serial.print("**** ERROR MS5611 : ");
    Serial.println(error);
    delay(1000);
  }

  #define LED_PIN 2 // Turn LED on during setup. 13->Arduino, 2->ESP32
  
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  //FACTORY CALIBRATION
  //Every module is individually factory calibrated at two temperatures and two pressures. 
  //As a result, 6 coefficients necessary to compensate process and temperature variations are calculated and stored in the 128-bit PROM of each module. 
  //These bits (partitioned into 6 coefficients) must be read by the microcontroller software and used in the program converting D1 and D2 into compensated pressure and temperature values.
  
  //COMMANDS
  //The MS5611-01BA has only five basic commands:
  //  1. Reset
  //  2. Read PROM (128 bit of calibration words)
  //  //3. D1 conversion
  //  //4. D2 conversion
  //  5. Read ADC result (24 bit pressure / temperature)
  
  for ( uint8_t start = 1; start <= 6; start++) {
    Wire1.beginTransmission(MS5611_ADDRESS);      //Start communication with the MS5611
    Wire1.write(0xA0 + start * 2);                //Send the address that we want to read
    Wire1.endTransmission();                      //End the transmission
    Wire1.requestFrom(MS5611_ADDRESS, 2);         //Request 2 bytes from the MS5611
    C[start] = Wire1.read() << 8 | Wire1.read();  //Add the low and high byte to the C[x] calibration variable
    //Serial.println(C[start]);
  }

  OFF_C2 = C[2] * pow(2, 16);
  SENS_C1 = C[1] * pow(2, 15);
  
  //El MS5611 necesita de unas lecturas para estabilizarse
  unsigned long duracion = millis();
  
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  for ( uint16_t start = 1; start < 1000; start++) {
    //automata_MS5611(false); //flag a "false" para que no calcule PIDs
    delayMicroseconds(SAMPLETIME-100); //delay of 4us to simulate the 250Hz program loop
  }

  Serial.println("\nCalibracion MS5611:");
  Serial.println("--------------------");
  Serial.print("actual_pressure:"); Serial.println(actual_pressure);
  Serial.print("duracion calibracion: "); Serial.print((millis() - duracion)/1000); Serial.println(" segundos");
  
  digitalWrite(LED_PIN, LOW);

  //RESETs
  actual_pressure = 0;  
  barometer_counter = 0;
  temperature_counter = 0;
}



void loop() {
  //Serial.print(barometer_counter);Serial.print("\t");Serial.println(temperature_counter);

  barometer_counter++;

  //=================================== PASO 1 ===================================
  if (barometer_counter == 1) { 
    if (temperature_counter == 0) {
        //Get temperature data from MS5611.
        //----------------------------------
        Wire1.beginTransmission(MS5611_ADDRESS);   //Open a connection with the MS5611.
        Wire1.write(0x00);                         //Send a 0 to indicate that we want to poll the requested data.
        Wire1.endTransmission();                   //End the transmission with the MS5611.
        Wire1.requestFrom(MS5611_ADDRESS, 3);      //Poll 3 data bytes from the MS5611.
      
        //Store the temperature in a 5 location rotating memory to prevent temperature spikes.
        //------------------------------------------------------------------------------------
        raw_average_temperature_total -= raw_temperature_rotating_memory[average_temperature_mem_location];
        raw_temperature_rotating_memory[average_temperature_mem_location] = Wire1.read() << 16 | Wire1.read() << 8 | Wire1.read();
        raw_average_temperature_total += raw_temperature_rotating_memory[average_temperature_mem_location];
        average_temperature_mem_location++;
        if (average_temperature_mem_location == TEMPERATURE_ROTATING_MEMORY)average_temperature_mem_location = 0;
        raw_temperature = raw_average_temperature_total / TEMPERATURE_ROTATING_MEMORY; //Calculate the avarage temperature of the last 5 measurements.
    } else {
        //Get pressure data from MS5611.
        //------------------------------
        Wire1.beginTransmission(MS5611_ADDRESS);  //Open a connection with the MS5611.
        Wire1.write(0x00);                        //Send a 0 to indicate that we want to poll the requested data.
        Wire1.endTransmission();                  //End the transmission with the MS5611.
        Wire1.requestFrom(MS5611_ADDRESS, 3);     //Poll 3 data bytes from the MS5611.
        //Shift the individual bytes in the correct position and add them to the raw_pressure variable.
        raw_pressure = Wire1.read() << 16 | Wire1.read() << 8 | Wire1.read();
    }

    temperature_counter ++;  
    
    if (temperature_counter == 20) {
        temperature_counter = 0;    
        //Request temperature data.
        //-------------------------
        Wire1.beginTransmission(MS5611_ADDRESS);   //Open a connection with the MS5611.
        Wire1.write(0x58);                         //Send a 0x58 to indicate that we want to request the temperature data.
        Wire1.endTransmission();                   //End the transmission with the MS5611.
    } else {
        //Request pressure data.
        //----------------------
        Wire1.beginTransmission(MS5611_ADDRESS);   //Open a connection with the MS5611
        Wire1.write(0x48);                         //Send a 0x48 to indicate that we want to request the pressure data.
        Wire1.endTransmission();                   //End the transmission with the MS5611.
    }
  }

  //=================================== PASO 2 ===================================
  if (barometer_counter == 2) {
    //Calculate pressure as explained in the datasheet of the MS5611.
    //---------------------------------------------------------------
    dT = C[5];
    dT <<= 8;
    dT *= -1;
    dT += raw_temperature;
    OFF = OFF_C2 + ((int64_t)dT * (int64_t)C[4]) / pow(2, 7);
    SENS = SENS_C1 + ((int64_t)dT * (int64_t)C[3]) / pow(2, 8);
    P = ((raw_pressure * SENS) / pow(2, 21) - OFF) / pow(2, 15);
    
    //To get a smoother pressure value we will use a 20 location rotating memory.
    //---------------------------------------------------------------------------
    pressure_total_avarage -= pressure_rotating_mem[pressure_rotating_mem_location];                    //Subtract the current memory position to make room for the new value.
    pressure_rotating_mem[pressure_rotating_mem_location] = P;                                          //Calculate the new change between the actual pressure and the previous measurement.
    pressure_total_avarage += pressure_rotating_mem[pressure_rotating_mem_location];                    //Add the new value to the long term avarage value.
    pressure_rotating_mem_location++;                                                                   //Increase the rotating memory location.
    if (pressure_rotating_mem_location == PRESSURE_ROTATING_MEMORY)pressure_rotating_mem_location = 0;  //Start at 0 when the memory location 20 is reached.
    actual_pressure_fast = (float)pressure_total_avarage / (float)(1.0 * PRESSURE_ROTATING_MEMORY);     //Calculate the average pressure of the last 20 pressure readings.

    //To get better results we will use a complementary filter that can be adjusted by the fast average.
    //--------------------------------------------------------------------------------------------------
    actual_pressure_slow = actual_pressure_slow * (float)0.985 + actual_pressure_fast * (float)0.015;
    actual_pressure_diff = actual_pressure_slow - actual_pressure_fast;                                           //Calculate the difference between the fast and the slow avarage value.
    if (actual_pressure_diff > 8)actual_pressure_diff = 8;                                                        //If the difference is larger then 8 limit the difference to 8.
    if (actual_pressure_diff < -8)actual_pressure_diff = -8;                                                      //If the difference is smaller then -8 limit the difference to -8.
    if (actual_pressure_diff > 1 || actual_pressure_diff < -1)actual_pressure_slow -= actual_pressure_diff / 6.0; //If the difference is larger then 1 or smaller then -1 the slow average is adjuste based on the error between the fast and slow average.
    actual_pressure = actual_pressure_slow;                                                                       //The actual_pressure is used in the program for altitude calculations.
  }

  //=================================== PASO 3 ===================================
  if (barometer_counter == 3) {
    barometer_counter = 0;
    if (pid) {
      //In the following part a rotating buffer is used to calculate the long term change between the various pressure measurements.
      //This total value can be used to detect the direction (up/down) and speed of the quadcopter and functions as the D-controller of the total PID-controller.
    }
  }

}
