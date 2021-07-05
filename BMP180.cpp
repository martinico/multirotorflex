
#include <Arduino.h>
#include "BMP180.h"
#include "Wire.h"

#define BMP180_ADDRESS 0x77   //I2C address of the BMP180

#define SDA1 4
#define SCL1 5

#define TRAZAS_CONVERGENCIA 0

#define LED_PIN 2 // Turn LED on during setup. 13->Arduino, 2->ESP32

// Ultra Low Power       OSS = 0, OSD =  5ms (1 internal number of samples) 0.06=RMS noise typ. [hPa]
// Standard              OSS = 1, OSD =  8ms (2                           ) 0.05
// High                  OSS = 2, OSD = 14ms (4                           ) 0.04
// Ultra High Resolution OSS = 3, OSD = 26ms (8                           ) 0.03
const uint8_t OSS = 2;     // Set oversampling setting with ...
const uint8_t OSD = 14;    // ... the corresponding oversampling delay.

const int32_t DELAY_5 = 5;       //delay en milisegundos necesario tras pedir una temperatura
const int32_t DELAY_OSD = OSD;   //delay en milisegundos tras pedir una medida de presion

// STORED SENSOR VALUES IN THE PROM OF BMP180
// ******************************************
int16_t  ac1, ac2, ac3, b1, b2, mb, mc, md;
uint16_t ac4, ac5, ac6;

// temperatura "maquina" necesaria para el calculo de la presion
// *************************************************************
int32_t b5, bb55;

// presiones "maquina" necesaria para el calculo de la presion
// ***********************************************************
int32_t UT, UP, P, P_1;

float T, H;                                   // variables globales directas y medibles de temperatura, (presion) y altitud
float PP_guay, PP_fast, PP_slow, PP_diff;     // variables globales calculadas/filtradas de IDEM
float Pref, Href;                             // presion y altitud media calculada por iteraciones para la calibracion

float SEALEVEL = 1013.25;

//Constantes y variables declaradas en este modulo y exportables al MAIN
//----------------------------------------------------------------------
//float hGND, hREF, hQNH;
//int PIDaltimetro;

//CONTROL DEL CICLO
//-----------------
const uint8_t BUFFER_TEMPERATURAS = 3;
int32_t rotating_memory_temperaturas[BUFFER_TEMPERATURAS + 1];
uint8_t memory_location_temperaturas = 0;
int32_t temperaturas_medias = 0;
    
const uint8_t BUFFER_PRESIONES = 30;
int32_t rotating_memory_presiones[BUFFER_PRESIONES + 1];
uint8_t memory_location_presiones = 0;
int32_t presiones_medias = 0;

const uint32_t CICLOS = 1000;
uint16_t contador_temperaturas, contador_presiones;
uint32_t contador_ciclos;
const float FILTERpre = 0.985; //190988

const uint8_t BUFFER_PARACHUTE = 30;
float rotating_memory_parachute[BUFFER_PARACHUTE+1];
uint8_t memory_location_parachute;
float throttle_parachute, anterior_parachute;

//https://tablademareas.com/es/sevilla/sevilla/prevision/presion-atmosferica
const int32_t PRESION_MAXIMA = 103000; //pascales
const int32_t PRESION_MINIMA =  99000; //pascales



uint8_t read_1_byte(uint8_t code) {
  uint8_t value;
  Wire1.beginTransmission(BMP180_ADDRESS);         
  Wire1.write(code);                               
  Wire1.endTransmission();                         
  Wire1.requestFrom(BMP180_ADDRESS, 1);            
  if(Wire1.available() >= 1) {
    value = Wire1.read();                          
  }
  return value;                                   
}

uint16_t read_2_bytes(uint8_t code) {
  uint16_t value;
  Wire1.beginTransmission(BMP180_ADDRESS);         
  Wire1.write(code);                               
  Wire1.endTransmission();                         
  Wire1.requestFrom(BMP180_ADDRESS, 2);            
  if(Wire1.available() >= 2) {
    value = (Wire1.read() << 8) | Wire1.read();
  }
  return value;
}



void init_BMP180 () {
  unsigned long duracion = millis();
  
  Wire1.begin(SDA1, SCL1);
  Wire1.setClock(400000); // 100kHz I2C clock - comment this line if having compilation difficulties 
  Wire1.beginTransmission(BMP180_ADDRESS);
  uint8_t error = Wire1.endTransmission();
  while (error != 0) {
    Serial.print("**** ERROR BMP180 : ");
    Serial.println(error);
    delay(1000);
  }
  
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
    
  ac1 = read_2_bytes(0xAA);
  ac2 = read_2_bytes(0xAC);
  ac3 = read_2_bytes(0xAE);
  ac4 = read_2_bytes(0xB0);
  ac5 = read_2_bytes(0xB2);
  ac6 = read_2_bytes(0xB4);
  b1  = read_2_bytes(0xB6);
  b2  = read_2_bytes(0xB8);
  mb  = read_2_bytes(0xBA);
  mc  = read_2_bytes(0xBC);
  md  = read_2_bytes(0xBE);

  //Serial.print(F("AC1 = ")); Serial.println(ac1);
  //Serial.print(F("AC2 = ")); Serial.println(ac2);
  //Serial.print(F("AC3 = ")); Serial.println(ac3);
  //Serial.print(F("AC4 = ")); Serial.println(ac4);
  //Serial.print(F("AC5 = ")); Serial.println(ac5);
  //Serial.print(F("AC6 = ")); Serial.println(ac6);
  //Serial.print(F("B1 = "));  Serial.println(b1);
  //Serial.print(F("B2 = "));  Serial.println(b2);
  //Serial.print(F("MB = "));  Serial.println(mb);
  //Serial.print(F("MC = "));  Serial.println(mc);
  //Serial.print(F("MD = "));  Serial.println(md);

  Pref = 0.0;

  for (int cal = 0; cal < 200; cal++) {
    pedirTemperatura();
      //delay (DELAY_5);
      vTaskDelay(DELAY_5);
    leerTemperatura();
    calcularTemperatura();
    bb55 = b5; //truco

    pedirPresion();
      //delay(DELAY_OSD);
      vTaskDelay(DELAY_OSD);
    leerPresion();
    calcularPresion();
    
    Pref  += P;
  }

  Pref = Pref/200.0; //en pascales dividiendo por el contador
  Pref = Pref/100.0; //en milibares para que sea legible

  //Calculating altitude with reasonable accuracy requires the sea level pressure for your position at the moment the data is converted, 
  //as well as the ambient temperature in degress celcius. If you don't have these values, a 'generic' value of 1013.25 hPa can be used,
  //but this isn't ideal and will give variable results from one day to the next.
  //You can usually find the current SLP (SeaLevelPressure) value by looking at weather websites or from environmental information centers
  //near any major airport: https://www.tutiempo.net/Estaciones/Aeropuerto-de-Sevilla-San-Pablo/

  Href = 44330.0 * (1.0 - pow(Pref / SEALEVEL, 0.1903));

  Serial.println("Calibracion BMP180:");
  Serial.print("Temperatura: "); Serial.print(T, 3); Serial.println(" ºC");
  Serial.print("Presion Referencia: "); Serial.print(Pref, 3); Serial.print(" milibares - "); Serial.print(Pref * 0.75006375541921, 3); Serial.println(" mmHg");
  Serial.print("Altitud Referencia: "); Serial.print(Href, 3); Serial.println(" metros");
  Serial.print("duracion calibracion: "); Serial.print((millis() - duracion)/1000); Serial.println(" segundos");
  Serial.println("");

  digitalWrite(LED_PIN, LOW);

  reset_BMP180();

  pedirTemperatura(); //¡¡¡OJO!!!
}

void reset_BMP180 () { 
// 1 m -> 11.61 pascal || 1 pascal => 0.0861 m
// H = 1/2 * G * T**2             
// H = 1/2 * 9.81m/sec * (10medidas * 14msec/1000sec)**2 ) * 11.61 =  1,116 pascal
// H = 1/2 * 9.81m/sec * (20medidas * 14msec/1000sec)**2 ) * 11.61 =  4,465 pascal
// H = 1/2 * 9.81m/sec * (30medidas * 14msec/1000sec)**2 ) * 11.61 = 10,045 pascal
// H = 1/2 * 9.81m/sec * (40medidas * 14msec/1000sec)**2 ) * 11.61 = 17,856 pascal

  FILTERpos = 0.9985; //filtro standard tanto para el BMP180 como para el MS5611
  DESVIACION_PRESION_PERMITIDA = 4; //MAXIMUN ALLOWED PEAKS (pascal) -> sin uso
  
  //                     14msec
  DESVIACION_1G_1M =      4.0;    //5ms, 1sample, 6pascal | 8ms, 2sample, 5pascal | 14ms, 4sample, 4pascal | 26ms, 8sample, 3pascal
  DESVIACION_1G_MAX =    10.045;  //segun calculos < 10,045
  ATENUACION_FILTRO =     6.0;    //ATENUACION/AMPLIFICACION tanto para el BMP180 como para el MS5611
  AMPLIFICACION_FILTRO =  9.0;    //ATENUACION/AMPLIFICACION tanto para el BMP180 como para el MS5611

  contador_temperaturas = 0;
  contador_presiones = 0;
  contador_ciclos = 0;

  hGND = hREF = hQNH = 0.0;

  //reseteo de las variables de control
  throttle_parachute = 0;
  PIDaltimetro = 0;
}



void pedirTemperatura () {
  Wire1.beginTransmission(BMP180_ADDRESS);
  Wire1.write(0xf4);
  Wire1.write(0x2e);
  Wire1.endTransmission();  
}

void leerTemperatura() {
  UT = read_2_bytes(0xf6); // lectura del valor de la temperatura
}

void calcularTemperatura() {
  int32_t x1, x2;
  
  // calcular la temperatura real
  x1 = (UT - (int32_t)ac6) * (int32_t)ac5 >> 15;
  x2 = ((int32_t)mc << 11) / (x1 + (int32_t)md);
  b5 = x1 + x2; //***** temperatura "maquina" *****
  T  = (b5 + 8) >> 4;
  T = T / 10.0; // ***** temperatura en celsius *****
}

void  pedirPresion() {
  Wire1.beginTransmission(BMP180_ADDRESS);
  Wire1.write(0xf4);  // enviar la dirección de registro
  Wire1.write(0x34 + (OSS << 6));  // escribe los datos
  Wire1.endTransmission();  
}

void leerPresion() {    
  Wire1.beginTransmission(BMP180_ADDRESS);
  Wire1.write(0xf6);                         
  Wire1.endTransmission();
  
  Wire1.requestFrom(BMP180_ADDRESS, 3);      
  if(Wire1.available() >= 3) {
    UP = (((int32_t)Wire1.read() << 16) | ((int32_t)Wire1.read() << 8) | ((int32_t)Wire1.read())) >> (8 - OSS);
  }
}

void calcularPresion () {
  int32_t x1, x2, x3, b3, b6, p;
  uint32_t b4, b7; 
  
  //b6 = b5 - 4000; //*** OJO, el b5 deberia ser la presion calculada o cocinada  ***
  b6 = bb55 - 4000; //*** OJO, el b5 deberia ser la presion calculada o cocinada  ***
  
  x1 = (b2 * (b6 * b6 >> 12)) >> 11; 
  x2 = ac2 * b6 >> 11;
  x3 = x1 + x2;
  b3 = (((ac1 * 4 + x3) << OSS) + 2) >> 2;
  x1 = ac3 * b6 >> 13;
  x2 = (b1 * (b6 * b6 >> 12)) >> 16;
  x3 = ((x1 + x2) + 2) >> 2;
  b4 = (ac4 * (uint32_t)(x3 + 32768)) >> 15;
  b7 = ((uint32_t)UP - b3) * (50000 >> OSS);
  if (b7 < 0x80000000) { p = (b7 << 1) / b4; } else { p = (b7 / b4) << 1; } // p = b7 < 0x80000000 ? (b7 * 2) / b4 : (b7 / b4) * 2;
  x1 = (p >> 8) * (p >> 8);
  x1 = (x1 * 3038) >> 16;
  x2 = (-7357 * p) >> 16;
  //P = (p + ((x1 + x2 + 3791) >> 4)) / 100.0f; // presion en mbar
  P = (p + ((x1 + x2 + 3791) >> 4)); // presion en pascales
}



// -----------------------------------------------------
// METODO CLASICO SIMILAR AL IMPLEMENTADO PARA EL MS5611
// -----------------------------------------------------
void calcularAltitudes() {
    float FILTRO;

    if (contador_ciclos <= CICLOS) {
      contador_ciclos++;
      FILTRO = FILTERpre;
      hGND = 44330.0 * (1.0 - pow((PP_guay/100.0) / SEALEVEL, 0.1903));
    } else {
      hREF = 44330.0 * (1.0 - pow((PP_guay/100.0) / SEALEVEL, 0.1903));
      hQNH = hREF - hGND;
      FILTRO = FILTERpos;
    }
    
    if (contador_temperaturas == 0) {
          leerTemperatura();
          calcularTemperatura(); 
          
          temperaturas_medias -= rotating_memory_temperaturas[memory_location_temperaturas];
          rotating_memory_temperaturas[memory_location_temperaturas] = b5;
          temperaturas_medias += rotating_memory_temperaturas[memory_location_temperaturas];
          memory_location_temperaturas++;
          if (memory_location_temperaturas >= BUFFER_TEMPERATURAS) memory_location_temperaturas = 0;
          bb55 = (int32_t) temperaturas_medias / BUFFER_TEMPERATURAS;
    } else {
          leerPresion();
          P_1 = P; //tomamos el valor anterior valido (PP_1) en caso de desvios extremos
          calcularPresion();
          if (P > PRESION_MAXIMA || P < PRESION_MINIMA) P = P_1; //Limites PRESION_MAXIMA, PRESION_MINIMA
          
          //if (contador_ciclos > CICLOS) {     
          //  P_1 = P; //tomamos el valor anterior valido (PP_1) en caso de desvios extremos
          //  if (P > (PP_fast + DESVIACION_PRESION_PERMITIDA))
          //            P = PP_fast + DESVIACION_PRESION_PERMITIDA;
          //  else if (P < (PP_fast - DESVIACION_PRESION_PERMITIDA))
          //            P = PP_fast - DESVIACION_PRESION_PERMITIDA;
          //}

          presiones_medias -= rotating_memory_presiones[memory_location_presiones]; 
          rotating_memory_presiones[memory_location_presiones] = P;      
          presiones_medias += rotating_memory_presiones[memory_location_presiones]; 
          memory_location_presiones++;      
          if (memory_location_presiones >= BUFFER_PRESIONES) memory_location_presiones = 0; 

          PP_fast = (float) presiones_medias / BUFFER_PRESIONES; 
          PP_slow = PP_slow * FILTRO + PP_fast * (1.0 - FILTRO); //FILTRO
          PP_diff = PP_slow - PP_fast;  

          if (PP_diff > DESVIACION_1G_MAX) PP_diff = DESVIACION_1G_MAX;
          else if (PP_diff < (-1.0 * DESVIACION_1G_MAX)) PP_diff = -1.0 * DESVIACION_1G_MAX;

          //filtroSimple();
          //filtroDoble();
          filtroFuzzy();
          //parachute();
    }

    contador_temperaturas ++;

    if (contador_temperaturas == BUFFER_PRESIONES) {
          contador_temperaturas = 0;
          pedirTemperatura();
    } else {
          pedirPresion();
    }

    trazas();
}

void filtroSimple() {
   if (PP_diff > DESVIACION_1G_1M || PP_diff < (-1.0 * DESVIACION_1G_1M))
                PP_slow -= PP_diff / ATENUACION_FILTRO;

   PP_guay = PP_slow; //y finalmente "actual_pressure=PP_guay" es la variable que se usara para el calculo de altitudes
}

void filtroDoble() {
     if (PP_diff > DESVIACION_1G_1M || PP_diff < (-1.0 * DESVIACION_1G_1M)) {
              PP_slow -= PP_diff / AMPLIFICACION_FILTRO;
              PP_guay = PP_slow;
              return;
   } else {
              PP_slow -= PP_diff / ATENUACION_FILTRO;
              PP_guay = PP_slow;
              return;
   }
  
}

void filtroFuzzy() {
   const float BOUNDARY = 1.0;
   
   float amplitud, diferencia, relacion;

   //if ( PP_diff > (DESVIACION_1G_1M * BOUNDARY) || PP_diff < (-1.0 * (DESVIACION_1G_1M * BOUNDARY)) ) {}
   if (PP_diff > DESVIACION_1G_1M || PP_diff < (-1.0 * DESVIACION_1G_1M)) {
                //amplitud = DESVIACION_1G_MAX - (DESVIACION_1G_1M * BOUNDARY);
                amplitud = DESVIACION_1G_MAX - DESVIACION_1G_1M;
                //diferencia = DESVIACION_1G_MAX - abs(PP_diff);
                diferencia = DESVIACION_1G_MAX - PP_diff;
                //relacion = (diferencia/amplitud) * AMPLIFICACION_FILTRO;                
                relacion = (diferencia/amplitud) * ATENUACION_FILTRO;                

                PP_slow -= PP_diff / relacion;

                //Serial.print(amplitud);Serial.print("\t");Serial.print(diferencia);Serial.print("\t");Serial.print(relacion);Serial.print("\n");
   }

   //if (PP_diff > DESVIACION_1G_1M || PP_diff < (-1.0 * DESVIACION_1G_1M)) PP_slow -= PP_diff / ATENUACION_FILTRO;
   PP_guay = PP_slow; //y finalmente "actual_pressure=PP_guay" es la variable que se usara para el calculo de altitudes  
}

void parachute() {
    //ALTITUDE HOLDING:
    // - se activara con la maquina "superestabilizada", o
    // - cuando activamos la funcion ALTITUDE HOLD (por implementar).
    if (contador_ciclos < CICLOS) return;
    if (contador_ciclos == CICLOS) anterior_parachute = PP_guay;
    
    //Durante un "manual_altitude_change" se deshabilita la detection del up/down.
    //if (manual_altitude_change) anterior_parachute = PP_guay;

    //Usando un buffer circular calcularemos las desviaciones "naturales" de las presiones debidas a los cambios de temperatura.
    //Este valor calculado podra usarse para:
    //    1) detectar la direccion del movimiento (up/down), 
    //    2) y el parametro D del control PID en altitud.

    throttle_parachute -= rotating_memory_parachute[memory_location_parachute];

    rotating_memory_parachute[memory_location_parachute] = PP_guay - anterior_parachute;

    throttle_parachute += rotating_memory_parachute[memory_location_parachute];

    anterior_parachute = PP_guay; //para el siguiente loop
    
    memory_location_parachute++;
  
    if (memory_location_parachute >= BUFFER_PARACHUTE) {
          memory_location_parachute = 0;
          //Serial.println(throttle_parachute);
    }
    
    //Serial.print("\t");Serial.println(throttle_parachute);
}

int PIDparachute() {
  return 0;
}

void trazas() {
    if (FiltroFusionSensoresPRE) {
          //Desviaciones y correciones de las presiones "brutas" respecto a una media.
          //Serial.print(P_1 - 100000); Serial.print("\t");
          Serial.print(P - 100000); Serial.print("\t");
          Serial.print(PP_fast - 100000.0); Serial.print("\t");
    }

    if (FiltroFusionSensoresPOS) {
          //Visualizacion del alisamiento y de la capacidad de reaccion ante cambios.
          Serial.print(PP_fast - 100000.0); Serial.print("\t");
          Serial.print(PP_slow - 100000.0); Serial.print("\t");
          //Serial.print(((PP_slow + DESVIACION_1G_1M) - 100000.0)/10.0); Serial.print("\t"); //banda + 4 pascal
          //Serial.print(((PP_slow - DESVIACION_1G_1M) - 100000.0)/10.0); Serial.print("\t"); //banda - 4 pascal
          Serial.print((100.0 * Pref) - 100000.0); Serial.print("\t");
    }
    if (FiltroFusionSensoresPRE || FiltroFusionSensoresPOS) {
          Serial.println("");
    }

    #if TRAZAS_CONVERGENCIA
      //Aproximadamenta a las 1000 iteraciones alcanzamos a estabilidad.
      Serial.print(P); Serial.print("\t");
      Serial.print(PP_fast); Serial.print("\t");
      Serial.print(PP_slow); Serial.print("\t\t");      
      Serial.print(Pref); Serial.print("\t");
      Serial.print(contador_ciclos);
      Serial.println("");
    #endif  
}



/*
//Altitude PID variables
float pid_p_gain_altitude = 1.4;           //Gain setting for the altitude P-controller (default = 1.4).
float pid_i_gain_altitude = 0.2;           //Gain setting for the altitude I-controller (default = 0.2).
float pid_d_gain_altitude = 0.75;          //Gain setting for the altitude D-controller (default = 0.75).
int pid_max_altitude = 400;                //Maximum output of the PID-controller (+/-).

float pid_i_mem_altitude, pid_altitude_setpoint, pid_altitude_input, pid_output_altitude, pid_last_altitude_d_error;
uint8_t parachute_rotating_mem_location;
int32_t parachute_buffer[35], parachute_throttle;
float pressure_parachute_previous;
int32_t pressure_rotating_mem[50], pressure_total_avarage;
uint8_t pressure_rotating_mem_location;
float pressure_rotating_mem_actual;


    // In the following part a rotating buffer is used to calculate the long term change between the various pressure measurements.
    // This total value can be used to detect the direction (up/down) and speed of the quadcopter and functions as the D-controller of the total PID-controller.

    if (manual_altitude_change == 1)pressure_parachute_previous = actual_pressure * 10; //During manual altitude change the up/down detection is disabled.

    //Subtract the current memory position to make room for the new value.
    parachute_throttle -= parachute_buffer[parachute_rotating_mem_location];

    //Calculate the new change between the actual pressure and the previous measurement.
    parachute_buffer[parachute_rotating_mem_location] = actual_pressure * 10 - pressure_parachute_previous;

    //Add the new value to the long term avarage value.
    parachute_throttle += parachute_buffer[parachute_rotating_mem_location]; 
                                    
    //Store the current measurement for the next loop.
    pressure_parachute_previous = actual_pressure * 10;

    //Increase the rotating memory location.
    parachute_rotating_mem_location++;
    
    //Start at 0 when the memory location 20 is reached.
    if (parachute_rotating_mem_location == 30)parachute_rotating_mem_location = 0;


    //If the quadcopter is in altitude mode and flying.
    //-------------------------------------------------
    if (flight_mode >= 2 && takeoff_detected == 1) {
            if (pid_altitude_setpoint == 0) pid_altitude_setpoint = actual_pressure; //If not yet set, set the PID altitude setpoint.

            // When the throttle stick position is increased or decreased the altitude hold function is partially disabled. 
            // The manual_altitude_change variable will indicate if the altitude of the quadcopter is changed by the pilot.
            manual_altitude_change = 0;      //Preset the manual_altitude_change variable to 0.
            manual_throttle = 0;             //Set the manual_throttle variable to 0.

            //If the throtttle is increased above 1600us (60%).
            if (channel_3 > 1600) {
                    manual_altitude_change = 1;                     //Set the manual_altitude_change variable to 1 to indicate that the altitude is adjusted.
                    pid_altitude_setpoint = actual_pressure;        //Adjust the setpoint to the actual pressure value so the output of the P- and I-controller are 0.
                    manual_throttle = (channel_3 - 1600) / 3;       //To prevent very fast changes in hight limit the function of the throttle.
            }

            //If the throtttle is lowered below 1400us (40%).
            if (channel_3 < 1400) {
                    manual_altitude_change = 1;                     //Set the manual_altitude_change variable to 1 to indicate that the altitude is adjusted.
                    pid_altitude_setpoint = actual_pressure;        //Adjust the setpoint to the actual pressure value so the output of the P- and I-controller are 0.
                    manual_throttle = (channel_3 - 1400) / 5;       //To prevent very fast changes in hight limit the function of the throttle.
            }

            //Calculate the PID output of the altitude hold.
            pid_altitude_input = actual_pressure;                                //Set the setpoint (pid_altitude_input) of the PID-controller.
            pid_error_temp = pid_altitude_input - pid_altitude_setpoint;         //Calculate the error between the setpoint and the actual pressure value.

            // To get better results the P-gain is increased when the error between the setpoint and the actual pressure value increases.
            // The variable pid_error_gain_altitude will be used to adjust the P-gain of the PID-controller.
            pid_error_gain_altitude = 0;                                         //Set the pid_error_gain_altitude to 0.

            //If the error between the setpoint and the actual pressure is larger than 10 or smaller then -10.
            if (pid_error_temp > 10 || pid_error_temp < -10) {
                    pid_error_gain_altitude = (abs(pid_error_temp) - 10) / 20.0;     //The positive pid_error_gain_altitude variable is calculated based based on the error.
                    if (pid_error_gain_altitude > 3) pid_error_gain_altitude = 3;    //To prevent extreme P-gains it must be limited to 3.
            }

            // In the following section the I-output is calculated. It's an accumulation of errors over time.
            // The time factor is removed as the program loop runs at 250Hz.
            pid_i_mem_altitude += (pid_i_gain_altitude / 100.0) * pid_error_temp;
            if (pid_i_mem_altitude > pid_max_altitude) pid_i_mem_altitude = pid_max_altitude;
            else if (pid_i_mem_altitude < pid_max_altitude * -1) pid_i_mem_altitude = pid_max_altitude * -1;

            // In the following line the PID-output is calculated.
            // P = (pid_p_gain_altitude + pid_error_gain_altitude) * pid_error_temp.
            // I = pid_i_mem_altitude += (pid_i_gain_altitude / 100.0) * pid_error_temp (see above).
            // D = pid_d_gain_altitude * parachute_throttle.
            pid_output_altitude = (pid_p_gain_altitude + pid_error_gain_altitude) * pid_error_temp + pid_i_mem_altitude + pid_d_gain_altitude * parachute_throttle;

            //To prevent extreme PID-output the output must be limited.
            if (pid_output_altitude > pid_max_altitude) pid_output_altitude = pid_max_altitude;
            else if (pid_output_altitude < pid_max_altitude * -1) pid_output_altitude = pid_max_altitude * -1;
    }

    //If the altitude hold function is disabled some variables need to be reset to ensure a bumpless start when the altitude hold function is activated again.
    //-------------------------------------------------------------------------------------------------------------------------------------------------------
    else if (flight_mode < 2 && pid_altitude_setpoint != 0) {         //If the altitude hold mode is not set and the PID altitude setpoint is still set.
            pid_altitude_setpoint = 0;                                      //Reset the PID altitude setpoint.
            pid_output_altitude = 0;                                        //Reset the output of the PID controller.
            pid_i_mem_altitude = 0;                                         //Reset the I-controller.
            manual_throttle = 0;                                            //Set the manual_throttle variable to 0 .
            manual_altitude_change = 1;                                     //Set the manual_altitude_change to 1.
    }
*/
