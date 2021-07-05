
#include <Arduino.h>
#include "MS5611.h"
#include "Wire.h"

#define MS5611_ADDRESS 0x77   //I2C address of the MS5611

#define SDA1 4
#define SCL1 5

#define LED_PIN 2 // Turn LED on during setup. 13->Arduino, 2->ESP32

#define TRAZAS_CONVERGENCIA   0
#define TRAZAS_TEMPERATURAS   0
#define TRAZAS_ALTITUDES      0
#define TRAZAS_FAST_SLOW_DIFF 0

int FiltroFusionSensoresPRE, FiltroFusionSensoresPOS = 0;

const float FILTERpre = 0.985; //BAND-PASS FILTER = 0.985; //190988
float FILTERpos = 0.99985;

//https://www.daftlogic.com/sandbox-google-maps-find-altitude.htm
const float PRESION_STANDARD = 101325.0; //presion al nivel del mar

const float PASCALES_A_METROS = 0.0861; //a 25 grados centigrados (100 pascales = 1 milibar)
const float METROS_A_PASCALES = 11.614; //a 25 grados centigrados (100 pascales = 1 milibar)
//https://www.mide.com/air-pressure-at-altitude-calculator
//        101325-101500=175 =>  100
//40ºC    -15.82m               -9.04m
//35ºC    -15.57m               -8.89m
//30ºC    -15.32m               -8.75m
//25ºC    -15.06m               -8.61m    
//20ºC    -14.81m               -8.46m
//15ºC    -14.56m               -8.32m
//10ºC    -14.30m               -8.17m
// 5ºC    -14.05m               -8.03m
// 0ºC    -13.80m               -7.88m

//https://tablademareas.com/es/sevilla/sevilla/prevision/presion-atmosferica
const int64_t PRESION_MAXIMA = 103000; //pascales
const int64_t PRESION_MINIMA =  99000; //pascales
int64_t DESVIACION_PRESION_PERMITIDA = 4; //4 pascales por ciclo = 4 * 0,0861 metros cada 12 msec

// 1 m -> 11.61 pascal || 1 pascal => 0.0861 m
// H = 1/2 * G * T**2             
// H = 1/2 * 9.81m/sec * (10medidas * 12msec/1000sec)**2 = 0,07063 m * 11.61 =  0,82 pascal
// H = 1/2 * 9.81m/sec * (20medidas * 12msec/1000sec)**2 = 0,28253 m * 11.61 =  3,28 pascal
// H = 1/2 * 9.81m/sec * (30medidas * 12msec/1000sec)**2 = 0,63569 m * 11.61 =  7,38 pascal
// H = 1/2 * 9.81m/sec * (40medidas * 12msec/1000sec)**2 = 1,13012 m * 11.61 = 13,12 pascal

                                                                                             //valores originales
float DESVIACION_1G_1M = 1.0; // desviacion debido a la gravedad por medida y en pascales     //1.0
float DESVIACION_1G_MAX = 3.0;                                                                //8.0
float ATENUACION_FILTRO = 4.0;                                                                //6.0
float AMPLIFICACION_FILTRO = 4.0;                                                             //x.x

//Constantes y variables declaradas en este modulo y exportables al MAIN
//------------------------------------------------------------------------
float hGND, hREF, hQNH;
int PIDaltimetro;

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
const uint16_t CALIBRATION = 2000;



//------------------------------------------------------------------
//-------------------------- TEMPERATURAS --------------------------
//------------------------------------------------------------------
uint32_t rawTemperature;
const uint8_t MAX_TEMPERATURE_BUFFER = 6;
uint32_t raw_temperature_rotating_memory[MAX_TEMPERATURE_BUFFER+1];
uint32_t raw_average_temperature_total;
uint8_t average_temperature_memory_location;
uint32_t raw_temperature, filtered_temperature;



//---------------------------------------------------------------
//-------------------------- PRESIONES --------------------------
//---------------------------------------------------------------
uint32_t rawPressure;
const uint8_t MAX_PRESSURE_BUFFER = 40;
int32_t dT;
int64_t OFF, SENS, PP, PP_1;
int32_t pressure_rotating_memory[MAX_PRESSURE_BUFFER+1];
uint8_t pressure_rotating_memory_location;
int32_t pressure_total_avarage;
float actual_pressure, actual_pressure_slow, actual_pressure_fast, actual_pressure_diff;
float ground_pressure, reference_pressure; 



//---------------------------------------------------------
//-------------------------- PID --------------------------
//---------------------------------------------------------
const uint8_t MAX_PARACHUTE_BUFFER = 80;
int32_t parachute_rotating_memory[MAX_PARACHUTE_BUFFER+1];
uint8_t parachute_rotating_memory_location;
int32_t parachute_throttle;
float parachute_previous;



//----------------------------------------------------------------------
//-------------------------- METODOS GLOBALES --------------------------
//----------------------------------------------------------------------
void init_MS5611() {    
  start_MS5611();

  //RESET and reload-time
  reset_MS5611();
  vTaskDelay(1000);
  
  //El MS5611 necesita de unas lecturas para estabilizarse
  unsigned long duracion = millis();

  Serial.println("EEPROM MS5611:");

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
  Serial.println("");

  SENS_C1     = C[1] * pow(2, 15);  //Serial.print("SENS_C1:");     Serial.println(long(SENS_C1));
  OFF_C2      = C[2] * pow(2, 16);  //Serial.print("OFF_C2:");      Serial.println(long(OFF_C2));
  TCS_C3      = C[3] / pow (2, 8);  //Serial.print("TCS_C3:");      Serial.println(long(TCS_C3));
  TCO_C4      = C[4] / pow (2, 7);  //Serial.print("TCO_C4:");      Serial.println(long(TCO_C4));
  TREF_C5     = C[5] * pow (2, 8);  //Serial.print("TREF_C5:");     Serial.println(long(TREF_C5));
  TEMPSENS_C6 = C[6] / pow (2, 23); //Serial.print("TEMPSENS_C6:"); Serial.println(long(TEMPSENS_C6));

  /*
  for ( uint8_t start1 = 1; start1 < 6; start1++) {
      requestTemperature();
      vTaskDelay(12); //tardanza minima del ADC (ver DataSheet)
      getTemperature();
      
      for ( uint8_t start2 = 1; start2 < 40; start2++) {
          requestPressure();
          vTaskDelay(12); //tardanza minima del ADC (ver DataSheet)
          getPressure();        
      }
  }

  Serial.print("duracion calibracion: "); Serial.print((millis() - duracion)/1000); Serial.println(" segundos \n");
  */
  
  resetAltitude(); //RESETs

  requestTemperature(); //para iniciar el automata con "buen pie"
}

bool calculateAltitude() {    
    if (cycle_counter == 0 ) {
        pinMode(LED_PIN, OUTPUT);
        digitalWrite(LED_PIN, HIGH);     
    }
    
    if (cycle_counter < CALIBRATION) {
              cycle_counter++;
              hGND = convertAltitude(actual_pressure);
          
              #if TRAZAS_CONVERGENCIA
              //Calcular el numero de ciclos necesarios para alcanzar estabilidad en las medidas.
              Serial.print(raw_temperature);Serial.print("\t");
              Serial.print(actual_pressure);Serial.print("\t");
              Serial.print(hGND);Serial.print("\t\t");
              Serial.println(cycle_counter);
              #endif
    }
    
    if (cycle_counter == CALIBRATION ) {
                  cycle_counter++;
                  ground_pressure = actual_pressure;
                  hREF = convertAltitude(ground_pressure);
                  digitalWrite(LED_PIN, LOW);
                  Serial.println("TEMPERATURAS, PRESIONES Y ALTITUDES CALIBRADAS Y ESTABILIZADAS");
    }
    
    if (cycle_counter > CALIBRATION ) {
                      hGND = convertAltitude(actual_pressure);
                      hQNH = hGND - hREF;

                      #if TRAZAS_TEMPERATURAS
                      //Convergencia de la temperaturas "raw" (rapida) y "filtered" (lenta).
                      //Se visualizaran preferentemente en el plotter.
                      Serial.print(raw_temperature);Serial.print("\t");
                      Serial.println(filtered_temperature);
                      #endif

                      #if TRAZAS_ALTITUDES
                      //Estabilidad de la altitud.
                      //Se visualizaran preferentemente en el plotter.
                      Serial.println(hQNH);
                      #endif
          
                      #if TRAZAS_FAST_SLOW_DIFF
                      //Suavidad de las presiones y reacciones ante cambios.
                      //Se visualizaran preferentemente en el terminal.
                      Serial.print(actual_pressure_diff);   Serial.print("\t");
                      Serial.print(DESVIACION_1G_1M);       Serial.print("\t");
                      Serial.print(-1.0*DESVIACION_1G_1M);  Serial.print("\t");
                      Serial.print(DESVIACION_1G_MAX);      Serial.print("\t");
                      Serial.print(-1.0*DESVIACION_1G_MAX); Serial.print("\t");
                      if (actual_pressure_diff > DESVIACION_1G_MAX) Serial.print("****");
                      if (actual_pressure_diff > DESVIACION_1G_1M) Serial.print("*");
                      Serial.println("");
                      #endif

                      if (FiltroFusionSensoresPRE) {
                          //Desviaciones y correciones de las presiones "brutas" respecto a una media.
                          Serial.print((long) (PP_1 - 100000)); 
                          Serial.print("\t");
                          Serial.print((long) (PP - 100000)); 
                          Serial.print("\t");
                          Serial.print(actual_pressure_fast - 100000.0);
                      }
                      if (FiltroFusionSensoresPOS) {
                          //Visualizacion del alisamiento y de la capacidad de reaccion ante cambios.
                          Serial.print(actual_pressure_fast - 100000.0); Serial.print("\t");
                          Serial.print(actual_pressure_slow - 100000.0); Serial.print("\t");
                          Serial.print((actual_pressure_slow+DESVIACION_1G_1M) - 100000.0); Serial.print("\t"); //banda + 1 pascal
                          Serial.print((actual_pressure_slow-DESVIACION_1G_1M) - 100000.0); Serial.print("\t"); //banda - 1 pascal
                      }
                      if (FiltroFusionSensoresPRE || FiltroFusionSensoresPOS) {
                          Serial.println("");
                      }
    }

    if (temperature_counter == 0) {
          calcTemperature();
    } else {
          calcPressure();
          calcParachute();
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

  //formulas viejas empleadas que adjunto simplemente por comparar
  //H = 14536.45 * ( 1.0 - pow ( P/PRESION_STANDARD, 0.190284 ) ); //pies
  //H = 44330.0 * ( 1.0 - pow ( P/PRESION_STANDARD, 0.1903 ) ); //metros

  //https://www.mide.com/air-pressure-at-altitude-calculator
  /*
  H = Hb + Tb/Lb * ( pow ( (P/Pb), (-R*Lb)/(G*M) ) - 1 );
  H = 0 + (273+21.5)/(-0.0065) * ( pow ( (P/1013.25), (-8.31432*-0.0065/9.80665*0.00289644) ) - 1);
  H = -45307.69 * ( pow ( P/101325, 1.9062 ) - 1 );
  
  P  = static pressure about sea level [Pa]
  Pb = static pressure at sea level [Pa]
  Tb = standard temperature (temperature at sea level) [K]
  Lb = standard temperature lapse rate [K/m] = -0.0065 [K/m]
  H  = height about sea level [m]
  Hb = height at the bottom of atmospheric layer [m]
  R  = universal gas constant = 8.31432
  G  = gravitational acceleration constant = 9.80665
  M  = molar mass of Earth’s air = 0.0289644 [kg/mol]
  */

  H = (PRESION_STANDARD - P) / (PASCALES_A_METROS * 100.0);

  H= filterAltitude(H);

  return H;
}

float filterAltitude(float P) {
  float R;
  // R = ceilf (P*10.0)/10.0; //conversion a un solo digito decimal
  // R = floorf(P*10.0)/10.0; //conversion a un solo digito decimal
  // R = truncf(P*10.0)/10.0; //conversion a un solo digito decimal
     R = roundf(P*10.0)/10.0; //conversion a un solo digito decimal
     
  return R;
}

void resetAltitude () {
  actual_pressure = 0.0;
  ground_pressure = 0.0;
  
  barometer_counter = 0;
  temperature_counter = 0;
  cycle_counter = 0;

  hGND = hREF = hQNH = 0.0;

  PIDaltimetro = 0;
}

int parachutePID() {
  //if (pid_altitude_setpoint == 0) pid_altitude_setpoint = actual_pressure; //if not yet set, set the PID altitude setpoint
  
  //P = (pid_p_gain_altitude + pid_error_gain_altitude) * pid_error_temp
  //I = pid_i_mem_altitude += (pid_i_gain_altitude / 100.0) * pid_error_temp
  //D = pid_d_gain_altitude * parachute_throttle

  return 0;
}



//---------------------------------------------------------------------
//-------------------------- METODOS LOCALES --------------------------
//---------------------------------------------------------------------
void start_MS5611() {    
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

int reset_MS5611() {
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

void calcTemperature() {
    getTemperature();
          
    //Vamos guardando las temperaturas en el buffer rotatorio "raw_temperature_rotating_memory"
    //Calculamos, restando el valor antiguo y sumando el nuevo, el "raw_average_temperature_total"
    //Actualizamos el indice "average_temperature_memory_location" del buffer rotatorio
    //Calculamos la temperatura media "raw_temperature" del buffer rotatorio          
    raw_average_temperature_total -= raw_temperature_rotating_memory[average_temperature_memory_location];
    raw_temperature_rotating_memory[average_temperature_memory_location] = rawTemperature;
    raw_average_temperature_total += raw_temperature_rotating_memory[average_temperature_memory_location];          
    average_temperature_memory_location++;
    if (average_temperature_memory_location >= MAX_TEMPERATURE_BUFFER) 
                average_temperature_memory_location = 0;
    raw_temperature = raw_average_temperature_total / MAX_TEMPERATURE_BUFFER;

    //Vamos a usar un FILTRO COMPLEMENTARIO para que alise la "raw_temperature" en una alisada "filtered_temperature"
    filtered_temperature = (uint32_t)(filtered_temperature * 0.85) + (uint32_t)(raw_temperature * 0.15);

}

void calcPressure() {
    uint32_t temperature;

    if (cycle_counter > CALIBRATION)
          temperature = filtered_temperature;
    else
          temperature = raw_temperature;

    getPressure();
          
    //Calcular presiones de la manera indicada en el datasheet del MS5611.
    dT = C[5];
    dT <<= 8;
    dT *= -1;
    //dT += raw_temperature;
    dT += temperature;
    OFF = OFF_C2 + ((int64_t)dT * (int64_t)C[4]) / pow(2, 7);
    SENS = SENS_C1 + ((int64_t)dT * (int64_t)C[3]) / pow(2, 8);
    //PP_1 = PP; //salvaguarda de la presion anterior = PP
    PP = ((rawPressure * SENS) / pow(2, 21) - OFF) / pow(2, 15);

    //Limites PRESION_MAXIMA, PRESION_MINIMA y DESVIACION_PRESION_PERMITIDA
    //Tomamos el valor anterior valido (PP_1) en caso de desvios extremos
    if (cycle_counter > CALIBRATION) {     
              PP_1 = PP; //salvaguarda de la presion anterior = PP
                           
              if (PP > (actual_pressure_fast + DESVIACION_PRESION_PERMITIDA))
                      PP = actual_pressure_fast + DESVIACION_PRESION_PERMITIDA;
              else if (PP < (actual_pressure_fast - DESVIACION_PRESION_PERMITIDA))
                      PP = actual_pressure_fast - DESVIACION_PRESION_PERMITIDA;
    }

    //Vamos guardando las prsiones en el buffer rotatorio "raw_pressure_rotating_memory"
    //Calculamos, restando el valor antiguo y sumando el nuevo, el "raw_average_pressure_total"
    //Actualizamos el indice "average_pressure_memory_location" del buffer rotatorio
    pressure_total_avarage -= pressure_rotating_memory[pressure_rotating_memory_location];      
    pressure_rotating_memory[pressure_rotating_memory_location] = PP;      
    pressure_total_avarage += pressure_rotating_memory[pressure_rotating_memory_location];      
    pressure_rotating_memory_location++;      
    if (pressure_rotating_memory_location >= MAX_PRESSURE_BUFFER)
                pressure_rotating_memory_location = 0;  

    //Vamos a usar un FILTRO COMPLEMENTARIO que ajuste/alise el "actual_pressure_fast"
    actual_pressure_fast = (float) pressure_total_avarage / (MAX_PRESSURE_BUFFER * 1.0);
    if (cycle_counter > CALIBRATION)
                actual_pressure_slow = actual_pressure_slow * FILTERpos + actual_pressure_fast * (1.0 - FILTERpos);
    else
                actual_pressure_slow = actual_pressure_slow * FILTERpre + actual_pressure_fast * (1.0 - FILTERpre);

    //Calculate the difference between the fast and the slow avarage value.
    actual_pressure_diff = actual_pressure_slow - actual_pressure_fast;  
    if (actual_pressure_diff > DESVIACION_1G_MAX) 
                actual_pressure_diff = DESVIACION_1G_MAX;
    else if (actual_pressure_diff < (-1.0*DESVIACION_1G_MAX)) 
                actual_pressure_diff = -1.0*DESVIACION_1G_MAX;
      
    //Si la diferencia es mayor que +1 o menor que -1, "actual_pressure_slow" se ajusta con una porcion de dicha diferencia.
    if (actual_pressure_diff > DESVIACION_1G_1M || actual_pressure_diff < (-1.0 * DESVIACION_1G_1M))
                actual_pressure_slow -= actual_pressure_diff / ATENUACION_FILTRO;

    //Y finalmente "actual_pressure" es la variable que se usara para el calculo de altitudes.
    actual_pressure = actual_pressure_slow;
}

void calcParachute () {
  //In the following part a rotating buffer is used to calculate the long term change between the various pressure measurements.
  //This total value can be used to :
  //    1) detect the direction (up/down) and speed of the quadcopter,
  //    2) and functions like the D-controller of the total PID-controller.
  
  //if (manual_altitude_change) parachute_previous = actual_pressure * 10; //during manual altitude change the up/down detection is disabled

  //subtract the current memory position to make room for the new value
  parachute_throttle -= parachute_rotating_memory[parachute_rotating_memory_location];
  //calculate the new change between the actual pressure and the previous measurement
  parachute_rotating_memory[parachute_rotating_memory_location] = actual_pressure * 10 - parachute_previous;
  //add the new value to the long term avarage value
  parachute_throttle += parachute_rotating_memory[parachute_rotating_memory_location];
  //store the current measurement for the next loop
  parachute_previous = actual_pressure * 10;       
  //increase the rotating memory location
  parachute_rotating_memory_location++;
  //start at 0 when the maximum memory location is reached
  if (parachute_rotating_memory_location >= MAX_PARACHUTE_BUFFER) parachute_rotating_memory_location = 0;

  //P = (pid_p_gain_altitude + pid_error_gain_altitude) * pid_error_temp
  //I = pid_i_mem_altitude += (pid_i_gain_altitude / 100.0) * pid_error_temp (see above)
  //D = pid_d_gain_altitude * parachute_throttle

}
