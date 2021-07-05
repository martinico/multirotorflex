
#include <Arduino.h>
#include "MS5611.h"
#include "Wire.h"

#define MS5611_ADDRESS 0x77   //I2C address of the MS5611

#define SDA1 4
#define SCL1 5

#define TRAZAS_MS5611 1

const float FILTER = 0.985; //BAND-PASS FILTER
const float METROS_A_MILIBARES = 0.117; //delta(1 metro) => delta(0.117 milibar)
const float PRESION_STANDARD = 1013.25; //presion al nivel del mar

//https://tablademareas.com/es/sevilla/sevilla/prevision/presion-atmosferica
const int64_t PRESION_MAXIMA = 103000; //microbares
const int64_t PRESION_MINIMA = 99000; //microbares

// 1m -> 0.117mb
// 10m (g=9.81 m/s) => x = 1.17mb
// ------------------------------
// 1000msec -> 1.17mb
// 10msec -> x = 0.0117mb
const float DESVIACION_1G_20M = 3.51; //0.0117(valor) * 100(microbates) * 20(medidas);
const float DESVIACION_1G_1M = 1.17; //0.0117(valor) * 100(microbates) * 1(medidas);
const float ATENUACION_FILTRO = 5.0;

float hGND, hREF;

uint16_t C[7]; //obviamos el "0" 
/*
C1 Pressure sensitivity | SENST1
C2 Pressure offset | OFFT1
C3 Temperature coefficient of pressure sensitivity | TCS
C4 Temperature coefficient of pressure offset | TCO
C5 Reference temperature | TREF
C6 Temperature coefficient of the temperature | TEMPSENS
*/
int64_t SENS_C1, OFF_C2, TCS_C3, TCO_C4, TREF_C5, TEMPSENS_C6;

uint16_t barometer_counter, temperature_counter, cycle_counter;



//------------------------------------------------------------------
//-------------------------- TEMPERATURAS --------------------------
//------------------------------------------------------------------
uint32_t rawTemperature;
const uint8_t MAX_TEMPERATURE_BUFFER = 5;
uint32_t raw_temperature_rotating_memory[MAX_TEMPERATURE_BUFFER+1];
uint32_t raw_average_temperature_total;
uint8_t average_temperature_memory_location;
uint32_t raw_temperature;



//---------------------------------------------------------------
//-------------------------- PRESIONES --------------------------
//---------------------------------------------------------------
uint32_t rawPressure;
const uint8_t MAX_PRESSURE_BUFFER = 20;
int32_t dT;
int64_t OFF, SENS, PP, PP_1;
int32_t pressure_rotating_memory[MAX_PRESSURE_BUFFER+1];
uint8_t pressure_rotating_memory_location;
int32_t pressure_total_avarage;
float actual_pressure, actual_pressure_slow, actual_pressure_fast, actual_pressure_diff;



//---------------------------------------------------------
//-------------------------- PID --------------------------
//---------------------------------------------------------



//----------------------------------------------------------------------
//-------------------------- METODOS GLOBALES --------------------------
//----------------------------------------------------------------------
void initMS5611() {    
  startMS5611(); 
  delay (10);
  resetMS5611(); 
  delay (10);

  #define LED_PIN 2 // Turn LED on during setup. 13->Arduino, 2->ESP32
  
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  //El MS5611 necesita de unas lecturas para estabilizarse
  unsigned long duracion = millis();

  Serial.println("\nEEPROM MS5611:");
  Serial.println("------------------");

  //FACTORY CALIBRATION
  //Every module is individually factory calibrated at two temperatures and two pressures. 
  //As a result, 6 coefficients necessary to compensate process and temperature variations are calculated and stored in the 128-bit PROM of each module.
  //These bits (partitioned into 6 coefficients) must be read by the microcontroller software and used in the program converting D1 and D2 into compensated pressure and temperature values.
  
  //COMMANDS
  //The MS5611 has only five basic commands:
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
    Serial.print("C");Serial.print("[");Serial.print(start);Serial.print("]");Serial.print("=");Serial.println(C[start]);
  }

  SENS_C1     = C[1] * pow(2, 15);  //Serial.print("SENS_C1:");     Serial.println(long(SENS_C1));
  OFF_C2      = C[2] * pow(2, 16);  //Serial.print("OFF_C2:");      Serial.println(long(OFF_C2));
  TCS_C3      = C[3] / pow (2, 8);  //Serial.print("TCS_C3:");      Serial.println(long(TCS_C3));
  TCO_C4      = C[4] / pow (2, 7);  //Serial.print("TCO_C4:");      Serial.println(long(TCO_C4));
  TREF_C5     = C[5] * pow (2, 8);  //Serial.print("TREF_C5:");     Serial.println(long(TREF_C5));
  TEMPSENS_C6 = C[6] / pow (2, 23); //Serial.print("TEMPSENS_C6:"); Serial.println(long(TEMPSENS_C6));

  /*
  for ( uint8_t start = 1; start < 200; start++) {
      requestTemperature();
      delay(10); //tardanza minima del ADC (ver DataSheet)
      
      getTemperature();
      delay(10); //delay de 4us que simulan el buclue de 250Hz

      requestPressure();
      delay(10); //tardanza minima del ADC (ver DataSheet)
      
      getPressure();
      delay(10); //delay de 4us que simulan el buclue de 250Hz
  }
  */

  Serial.print("duracion calibracion: "); Serial.print((millis() - duracion)/1000); Serial.println(" segundos");
  
  digitalWrite(LED_PIN, LOW);

  resetAltitude(); //RESETs

  //para iniciar el automata con "buen pie"
  requestTemperature();
  delay(10);
}

bool calculateAltitude() {
    if (cycle_counter <= 2000) {
          cycle_counter++;
          hREF = convertAltitude(actual_pressure/100.0);
    } else {
          hGND = convertAltitude(actual_pressure/100.0);
          #if TRAZAS_MS5611
              Serial.print(actual_pressure_fast - 100000.0);Serial.print(",");
              Serial.print(actual_pressure_slow - 100000.0);Serial.print(",");
              Serial.println(PP  - 100000.0);
        
              //Serial.print(hGND);Serial.print(",");
              //Serial.print(hREF);Serial.print(",");
              //Serial.println(hGND-hREF);
          #endif
      }
  
      if (temperature_counter == 0) {
          getTemperature();
          
          //Vamos guardando las temperaturas en el buffer rotatorio "raw_temperature_rotating_memory"
          //Calculamos, restando el valor antiguo y sumando el nuevo, el "raw_average_temperature_total"
          //Actualizamos el indice "average_temperature_memory_location" del buffer rotatorio
          //Calculamos la temperatura media "raw_temperature" del buffer rotatorio          
          raw_average_temperature_total -= raw_temperature_rotating_memory[average_temperature_memory_location];
          raw_temperature_rotating_memory[average_temperature_memory_location] = rawTemperature;
          raw_average_temperature_total += raw_temperature_rotating_memory[average_temperature_memory_location];
          average_temperature_memory_location++;
          if (average_temperature_memory_location == MAX_TEMPERATURE_BUFFER) 
                average_temperature_memory_location = 0;
          raw_temperature = raw_average_temperature_total / MAX_TEMPERATURE_BUFFER;
      } else {
          getPressure();
          
          //Calcular presiones de la manera indicada en el datasheet del MS5611.
          dT = C[5];
          dT <<= 8;
          dT *= -1;
          dT += raw_temperature;
          OFF = OFF_C2 + ((int64_t)dT * (int64_t)C[4]) / pow(2, 7);
          SENS = SENS_C1 + ((int64_t)dT * (int64_t)C[3]) / pow(2, 8);
          PP_1 = PP; //salvaguarda del PP anterior
          PP = ((rawPressure * SENS) / pow(2, 21) - OFF) / pow(2, 15);
          
          /*
          //Limites PRESION_MAXIMA / PRESION_MINIMA
          if (cycle_counter > 2000)
            if (PP > PRESION_MAXIMA || PP < PRESION_MINIMA) 
              PP = PP_1;
          */
          
          //Calculo de una presion alisada simililar a la realizada con las temperaturas.
          pressure_total_avarage -= pressure_rotating_memory[pressure_rotating_memory_location];      
          pressure_rotating_memory[pressure_rotating_memory_location] = PP;      
          pressure_total_avarage += pressure_rotating_memory[pressure_rotating_memory_location];      
          pressure_rotating_memory_location++;      
          if (pressure_rotating_memory_location == MAX_PRESSURE_BUFFER)
                pressure_rotating_memory_location = 0;  
                    
          //Vamos a usar un FILTRO COMPLEMENTARIO que ajuste/alise el "actual_prssure_fast"
          actual_pressure_fast = (float) pressure_total_avarage / (MAX_PRESSURE_BUFFER * 1.0);
          actual_pressure_slow = actual_pressure_slow * FILTER + actual_pressure_fast * (1.0 - FILTER);
      
          //Calculate the difference between the fast and the slow avarage value.
          actual_pressure_diff = actual_pressure_slow - actual_pressure_fast;  
          if (actual_pressure_diff > DESVIACION_1G_20M) actual_pressure_diff = DESVIACION_1G_20M;   //Si la diferencia es mayor que +8 la limitamos a +8
          if (actual_pressure_diff < (-1.0*DESVIACION_1G_20M)) actual_pressure_diff = -1.0*DESVIACION_1G_20M;   //Si la diferencia es menor que -8 la limitamos a -8
      
          //Si la diferencia es mayor que +1 o menor que -1, "actual_pressure_slow" se ajusta con una porcion de dicha diferencia.
          if (actual_pressure_diff > DESVIACION_1G_1M || actual_pressure_diff < (-1.0*DESVIACION_1G_20M))
                actual_pressure_slow -= actual_pressure_diff / ATENUACION_FILTRO;

          //Y finalmente "actual_pressure" es la variables que se usara para el calculo de altitudes.
          actual_pressure = actual_pressure_slow;
      }

      temperature_counter ++;
      if (temperature_counter == MAX_PRESSURE_BUFFER) {
          temperature_counter = 0;
          requestTemperature();
      } else {
          requestPressure();
      }
  
}

float convertAltitude(float P) {
  float H;
  //H = 14536.45 * ( 1.0 - pow ( P/PRESION_STANDARD, 0.190284 ) );
  H = 44330.0 * ( 1.0 - pow ( P/PRESION_STANDARD, 0.1903 ) );
  return H;
}

void resetAltitude () {
  actual_pressure = 0.0;
  
  barometer_counter = 0;
  temperature_counter = 0;
  cycle_counter = 0;

  hGND = hREF = 0.0;
}



//---------------------------------------------------------------------
//-------------------------- METODOS LOCALES --------------------------
//---------------------------------------------------------------------
void startMS5611() {    
  Wire1.begin(SDA1, SCL1);                  // activa el bus I2C secundario
  Wire1.setClock(400000);                   // reloj = 100kHz (comenta la sentencia si encuentras dificultades)
  Wire1.beginTransmission(MS5611_ADDRESS);
  uint8_t error = Wire1.endTransmission();
  while (error != 0) {
    Serial.print("**** ERROR MS5611 : ");
    Serial.println(error);
    delay(1000);
  }
}

int resetMS5611() {
  Wire.beginTransmission(MS5611_ADDRESS);
  Wire.write(0x1E);
  return Wire.endTransmission();
}

void requestTemperature() {
  Wire1.beginTransmission(MS5611_ADDRESS);   //Open a connection with the MS5611.
  Wire1.write(0x58);                         //Send a 0x58 to indicate that we want to request the temperature data.
  Wire1.endTransmission();                   //End the transmission with the MS5611.
}

void getTemperature() {
  Wire1.beginTransmission(MS5611_ADDRESS);   //Open a connection with the MS5611.
  Wire1.write(0x00);                         //Send a 0 to indicate that we want to poll the requested data.
  Wire1.endTransmission();                   //End the transmission with the MS5611.
  Wire1.requestFrom(MS5611_ADDRESS, 3);      //Poll 3 data bytes from the MS5611.
  //Shift the individual bytes in the correct position and add them to the raw_pressure variable
  rawTemperature = Wire1.read() << 16 | Wire1.read() << 8 | Wire1.read();
  //Serial.print("T=");Serial.println(rawTemperature);
}

void requestPressure() {
  Wire1.beginTransmission(MS5611_ADDRESS);   //Open a connection with the MS5611
  Wire1.write(0x48);                         //Send a 0x48 to indicate that we want to request the pressure data.
  Wire1.endTransmission();                   //End the transmission with the MS5611.
}

void getPressure() {
  Wire1.beginTransmission(MS5611_ADDRESS);  //Open a connection with the MS5611.
  Wire1.write(0x00);                        //Send a 0 to indicate that we want to poll the requested data.
  Wire1.endTransmission();                  //End the transmission with the MS5611.
  Wire1.requestFrom(MS5611_ADDRESS, 3);     //Poll 3 data bytes from the MS5611.
  //Shift the individual bytes in the correct position and add them to the raw_pressure variable
  rawPressure = Wire1.read() << 16 | Wire1.read() << 8 | Wire1.read();
  //Serial.print("P=");Serial.println(rawPressure);
}
