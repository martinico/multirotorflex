
/*

**********************************
********** proyecto_2.3 **********
**********************************

Descripcion: Version avanzada del "multirotorflex"
Autor: Martin Schweighart Moya
Creacion: Sevilla, 3 de Febrero 2021

https://dl.espressif.com/dl/package_esp32_index.json
http://arduino.esp8266.com/stable/package_esp8266com_index.json
https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json

Chip is ESP32D0WDQ6 (revision 1)
Features: WiFi, BT, Dual Core, 240MHz, VRef calibration in efuse, Coding Scheme None
MAC: cc:50:e3:ab:ad:0c
Auto-detected Flash size: 4MB
802.11mc (Fine Timing Measurement) FMT to measure distance to AP

*/

#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include "globales.h"
#include "WEB.h"
#include "WEB2.h"
#include "MPU6050.h"
#include "HMC5883L.h"
//#include "MS5611.h"
#include "BMP180.h"
#include "GPS.h"
#include "PIDcontrol.h"
#include "propulsion.h"
//#include "kalman.h"
//#include "filtroC.h"

//#include "NRF24.h"

WebServer server(80); // http://192.168.4.1 (por defecto) o http://10.0.0.1 (nueva direccion)

WebSocketsServer websocket1 = WebSocketsServer (81); //MANDO
WebSocketsServer websocket2 = WebSocketsServer (82); //SENSORES

/*
Cambios efectuados en las librerias de Arduino para mejorar los WebSockets:
https://github.com/Links2004/arduinoWebSockets/blob/master/src/WebSockets.h
C:\Users\marti\Documents\Arduino\libraries\WebSockets\src\WebSockets.h
linea 58 : #define WEBSOCKETS_MAX_DATA_SIZE (20 * 1024)
linea 79 : #define WEBSOCKETS_TCP_TIMEOUT (1000)
*/

TaskHandle_t Task0, Task1, Task2, Task3, Task4, Task5, Task6;

SemaphoreHandle_t mutexI2C, mutexWS1, mutexWS2, mutexPOS;

SemaphoreHandle_t binario;

const uint16_t FREQUENCY = 250; //Hz del bucle principal
const uint16_t SAMPLETIME = 4000; //1000000/FRECUENCIA µs


 
//*********************************************************************
//**************** TAREA SENSORES (MPU6050 y HMC5883L) ****************
//*********************************************************************
void loop1 (void *parameter) {
  TickType_t retardo = pdMS_TO_TICKS(4);
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = retardo; // ¿dividido por portTICK_PERIOD_MS o portTICKRATEMS que equivalen a 1?
  //xLastWakeTime = xTaskGetTickCount(); // esta sentencia se ubicara al principio del bucle "for"

  #define STATUS_DE_VUELO 0

  unsigned long contador_de_tiempos = 0;
  
  unsigned long contador, acumulado, now, maximo, minimo = 0;
  
  int referencia; //PWM a comparar

  int PIDpitch, PIDroll, PIDyaw, PIDcompas, PIDgps; //PIDaltimetro definido en el modulo MS5611
 
  Serial.println("Tarea SENSORES 250Hz (MPU6050 y HMC5883L)");
  Serial.println("-----------------------------------------");

  //xSemaphoreTake( mutexI2C, portMAX_DELAY );
  init_MPU6050();
  init_HMC5883L(); //actual_compass_heading
  //xSemaphoreGive( mutexI2C );  

  sensoresCalibrados_250HZ = true;

  resetGyroAngles();
  setPIDtunings();
  resetPIDerrors();
  PIDpitch = PIDroll = PIDyaw = PIDcompas = PIDgps = 0;
  PIDaltimetro = 0; // definido en el modulo MS5611

  contador_de_tiempos = micros();

  xLastWakeTime = xTaskGetTickCount();// Initialise the xLastWakeTime variable with the current time.
  
  for (;;) {   
    vTaskDelayUntil( &xLastWakeTime, xFrequency ); // Wait for the next cycle.  
    //Serial.print("\t");Serial.println(micros() - contador_de_tiempos); //Tiempos de activacion de la tarea
    t1_250HZ = micros() - contador_de_tiempos;
    contador_de_tiempos = micros();   
    
    if (started) {
        resetGyroAngles();
        setPIDtunings();
        resetPIDerrors();
        PIDpitch = PIDroll = PIDyaw = PIDcompas, PIDgps = 0; 
        PIDaltimetro = 0; // definido en el modulo MS5611
        
        started = false;
        //Serial.println("started");
    }

    read_MPU6050_data(); 
    calculateAnglesMotion();
    HMC5883L_calculator; //actual_compass_heading
    t2_250HZ = micros() - contador_de_tiempos;

        //Procedimiento preciso para el control de tiempos 
        //Habilitable y trasladable al resto de las tareas
        if (tiemposBuclePrincipal_250HZ) {
            now = t2_250HZ;
            ++contador;
            acumulado += now;
            MEDIA_250HZ = (unsigned long) (acumulado / contador);
            //if  (now >  MAX_250HZ) MAX_250HZ = now;
            //if  (now <=  MIN_250HZ) MIN_250HZ = now;
        }

    //*********************************** ESTADO = volando || yawing || holding ***********************************
    if (volando) {
        //********** PITCH **********
        ( prop.joystickY != 0 ) ? referencia = prop.joystickY : referencia = SetpointPitch; 
        if (yawing) referencia = SetpointPitch;
        PIDpitch = (int) calculatePitchPID(referencia, angle_pitch_output, angular_motion_pitch); //*** velocidad + angulos
        //PIDpitch = (int) calculate2PitchPID(referencia, angle_pitch_output); //*** metodo tradicional
        //PIDpitch = (int) calculate3PitchPID(referencia, angle_pitch_output); //*** metodo moderno
        //PIDpitch = 0; //*** solo pruebas ***

        //********** ROLL **********
        ( prop.joystickX != 0 ) ? referencia = prop.joystickX : referencia = SetpointRoll;
        if (yawing) referencia = SetpointRoll;
        PIDroll = (int) calculateRollPID(referencia, angle_roll_output, angular_motion_roll); //*** velocidad + angulos
        //PIDroll = (int) calculate2RollPID(referencia, angle_roll_output); //*** metodo tradicional
        //PIDroll = (int) calculate3RollPID(referencia, angle_roll_output); //*** metodo moderno
        //PIDroll = 0; //*** solo pruebas ***

        //********** YAW **********
        if (!yawing){ //referencia = 0
              PIDyaw = (int) calculateYawPID(0, angle_yaw_output, angular_motion_yaw); //*** velocidad + angulos
              //PIDyaw = (int) calculate2YAWMotionPID(0, angular_motion_yaw); //based on the angular motion
        } else { //referencia <> 0        
              referencia = (int) calculateOnlyGyroPID(prop.joystickX);
              PIDyaw = (int) calculateYawPID(referencia, angle_yaw_output, angular_motion_yaw); //*** velocidad + angulos
              //PIDyaw = (int) calculate2YAWMotionPID(referencia, angular_motion_yaw); //based on the angular motion
              //PIDyaw = (int) calculate2YAWPositionPID(referencia, angle_yaw_output); //based on the angular position
        }
        //PIDyaw = 0; //*** solo pruebas ***

        //CONFIGURACION QuadX
        //prop.pwmPROA     = constrain ((prop.throttle+PIDaltimetro) - PIDpitch - PIDroll - PIDyaw, (prop.OFFSET+prop.UMBRAL), prop.LIMITE_ESC);
        //prop.pwmESTRIBOR = constrain ((prop.throttle+PIDaltimetro) + PIDpitch - PIDroll + PIDyaw, (prop.OFFSET+prop.UMBRAL), prop.LIMITE_ESC);
        //prop.pwmPOPA     = constrain ((prop.throttle+PIDaltimetro) + PIDpitch + PIDroll - PIDyaw, (prop.OFFSET+prop.UMBRAL), prop.LIMITE_ESC);
        //prop.pwmBABOR    = constrain ((prop.throttle+PIDaltimetro) - PIDpitch + PIDroll + PIDyaw, (prop.OFFSET+prop.UMBRAL), prop.LIMITE_ESC);

        //CONFIGURACION QuadPlus
        prop.pwmPROA     = constrain ((prop.throttle+PIDaltimetro) - PIDpitch + PIDyaw, (prop.OFFSET+prop.UMBRAL), prop.LIMITE_ESC); //CW
        prop.pwmPOPA     = constrain ((prop.throttle+PIDaltimetro) + PIDpitch + PIDyaw, (prop.OFFSET+prop.UMBRAL), prop.LIMITE_ESC); //CW
        prop.pwmESTRIBOR = constrain ((prop.throttle+PIDaltimetro) - PIDroll  - PIDyaw, (prop.OFFSET+prop.UMBRAL), prop.LIMITE_ESC); //CCW
        prop.pwmBABOR    = constrain ((prop.throttle+PIDaltimetro) + PIDroll  - PIDyaw, (prop.OFFSET+prop.UMBRAL), prop.LIMITE_ESC); //CCW
    }
    //*********************************** ESTADO = volando || yawing || holding ***********************************

    //*********************************** ESTADO = testing ***********************************
    else { //if (!volando) {
            prop.pwmPROA = prop.throttle;
            prop.pwmPOPA = prop.throttle;
            prop.pwmESTRIBOR = prop.throttle;
            prop.pwmBABOR = prop.throttle;
            if(!adjusted) {
              if (autoCalibracion_2()) {
                adjusted = true;
                newADJUST = true;
              }              
            }
    }
    //*********************************** ESTADO= testing ***********************************

    //while((micros() - contador_de_tiempos) < (SAMPLETIME - 10)) yield();

    if (!testing) {
        set_prop_ledc();
    }

    #if STATUS_DE_VUELO
        if (volando) {
            Serial.print("Volando");
            if (yawing)
                  Serial.print("\t Yawing");
            if (holding)
                  Serial.print("\t\t Holding");
            Serial.println("");
        } else {
            Serial.println("Testing");
        }
    #endif

    //vTaskDelay(retardo); //Cuando el "retardo" es nulo (vTaskDelay(0)=taskYIELD()) no pasamos al estado BLOCKED y si al READY
    //vTaskDelayUntil(&xLastWakeTime, xFrequency); //para que no salte un WARNING del WATCH-DOG TIMER
  }

}



//******************************************************************
//**************** TAREA SENSORES (MS5611 o BMP180) ****************
//******************************************************************
void loop2 (void *parameter) {
  TickType_t xLastWakeTime;
  //const TickType_t xFrequency = 12; //MS5611
  const TickType_t xFrequency = 12; //BMP180

  unsigned long contador_de_tiempos = 0;
  
  unsigned contador, acumulado, now, maximo, minimo = 0;
   
  Serial.println("Tarea SENSORES 100Hz (MS5611 o BMP180)");
  Serial.println("--------------------------------------");

  //xSemaphoreTake( mutexI2C, portMAX_DELAY );
  //init_MS5611();
  init_BMP180();
  //xSemaphoreGive( mutexI2C );  

  sensoresCalibrados_100HZ = true;

  contador_de_tiempos = micros();

  xLastWakeTime = xTaskGetTickCount();// Initialise the xLastWakeTime variable with the current time.
   
  for( ;; ) {
      vTaskDelayUntil( &xLastWakeTime, xFrequency ); // Wait for the next cycle.      
      //Serial.print("\t\t");Serial.println(micros() - contador_de_tiempos); //Tiempos de activacion de la tarea
      t1_100HZ = micros() - contador_de_tiempos;
      contador_de_tiempos = micros();     

      //calculateAltitude(); //MS5611
      calcularAltitudes(); //BMP180
      t2_100HZ = micros() - contador_de_tiempos;

          //Procedimiento preciso para el control de tiempos 
          //Habilitable y trasladable al resto de las tareas
          if (tiemposBuclePrincipal_100HZ) {
              now = t2_100HZ;
              ++contador;
              acumulado += now;
              MEDIA_100HZ = (unsigned long) (acumulado / contador);
              //if  (now >  MAX_100HZ) MAX_100HZ = now;
              //if  (now <=  MIN_100HZ) MIN_100HZ = now;
          }

      //*********************************** ESTADO = volando || yawing || holding ***********************************
      if (volando) {
        //PIDaltimetro = parachutePID(); //MS5611
        PIDaltimetro = PIDparachute(); //BMP180
      }
      //*********************************** ESTADO = volando || yawing || holding ***********************************

      //*********************************** ESTADO = testing ***********************************
      else { //if (!volando) {
            PIDaltimetro = 0;
      }
      //*********************************** ESTADO= testing ***********************************
      
  }

}



//******************************************************************
//************************** TAREA GPSNEO **************************
//******************************************************************
void loop3 (void *parameter) { 
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = 5000; //esta tarea entrara cada 5 segundos de SUSPEND a READY
  xLastWakeTime = xTaskGetTickCount();

  String ERROR = "{\"ERROR\":\"Error TX/RX GPSNEO\"}";

  Serial.print("Tarea GPSNEO \n");
  
  startGPSNEO(); //baudiosGPS, SERIAL_8N1, RXD2 & TXD2 

  for (;;) {
    if (!testGPSNEO(1000)) { //1000 milisegundos de espera hasta obtener un dato GPS valido
          jsonGPSNEO = ERROR;
          newGPSNEO = true;
    } else {
          jsonGPSNEO = "{\"GPSNEO\":{\"gps1\":";
          jsonGPSNEO += datosGPS.fecha;
          jsonGPSNEO +=  ",\"gps2\":";
          jsonGPSNEO += datosGPS.hora;
          jsonGPSNEO += ",\"gps3\":";
          jsonGPSNEO +=datosGPS.latitud;
          jsonGPSNEO += ",\"gps4\":";
          jsonGPSNEO += datosGPS.longitud;
          jsonGPSNEO += ",\"gps5\":";
          jsonGPSNEO += datosGPS.HDOP;
          jsonGPSNEO += ",\"gps6\":";
          jsonGPSNEO += datosGPS.altitud;
          jsonGPSNEO += ",\"gps7\":";
          jsonGPSNEO += datosGPS.VDOP;
          jsonGPSNEO += ",\"gps8\":";
          jsonGPSNEO += datosGPS.velocidad;
          jsonGPSNEO += ",\"gps9\":";
          jsonGPSNEO += datosGPS.direccion;
          jsonGPSNEO += ",\"gps10\":";
          jsonGPSNEO += datosGPS.satelites;  
          jsonGPSNEO += "}}";
          newGPSNEO = true;
    }

    //PIDgps = (int) calculateGPSPID(); //¡¡¡ y no queda ná !!!

    vTaskDelayUntil(&xLastWakeTime, xFrequency); //para que no salte un WARNING del WATCH-DOG TIMER
  }
  
}



//*********************************************************************
//********************* TAREA MISC (LiPO, WiFi, ...) ******************
//*********************************************************************
void loop4 (void *parameter) {
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = 10000; //esta tarea entrara cada 10 segundos de SUSPEND a READY
  xLastWakeTime = xTaskGetTickCount();

  Serial.print("Tarea LiPO, WiFi, ... \n");
  
  for (;;) {    
    jsonWIFI = "{\"WIFI\":";
    jsonWIFI += String(WiFi.RSSI());
    jsonWIFI += "}";
    newWIFI=true;

    //jsonWIFI = "{\"LIPO\":";
    //jsonWIFI += "xx,x";
    //jsonWIFI += "}";
    //newLIPO=true;
    
    vTaskDelayUntil(&xLastWakeTime, xFrequency); //para que no salte un WARNING del WATCH-DOG TIMER
  }  
  
}



//**************************************************************
//********************* TAREA = MENSAJERIA *********************
//**************************************************************
void loop5 (void *parameter) {
  TickType_t retardo = pdMS_TO_TICKS(200);
  
  String json;

  bool NL;

  Serial.print("Tarea MENSAJERIA \n");

  for (;;) {
    if (WS2conectado) {          
          json = "{\"MPU6050\":{";
          json += "\"pitch\":"; json += String(angle_pitch_output) + ",";
          json += "\"roll\":"; json += String(angle_roll_output) + ",";
          json += "\"yaw\":"; json += String(angle_yaw_output + actual_compass_heading); //Agulo calibrado con el compas.
          //json += "\"yaw\":"; json += String(angular_motion_yaw); //Solo en pruebas, y para analizar la tendencia de las fuerzas.
          json += "}}";
          //xSemaphoreTake( mutexWS2, portMAX_DELAY );
            websocket2.broadcastTXT (json.c_str(), json.length()); //MPU6050
          //xSemaphoreGive( mutexWS2 );

          json = "{\"MS5611\":";
          json += String(hQNH);
          json += "}";
          /*
          json = "{\"MS5611\":{";
          json += "\"ref\":"; json += String(hREF) + ",";
          json += "\"gnd\":"; json += String(hGND);
          json += "}}";
          */
          //xSemaphoreTake( mutexWS2, portMAX_DELAY );
            websocket2.broadcastTXT (json.c_str(), json.length()); //MS5611
          //xSemaphoreGive( mutexWS2 );
          
          if (newGPSNEO) {
              //xSemaphoreTake( mutexWS2, portMAX_DELAY );
                websocket2.broadcastTXT (jsonGPSNEO.c_str(), jsonGPSNEO.length()); 
              //xSemaphoreGive( mutexWS2 );
              newGPSNEO= false;           
          }
         
          if (newWIFI) {
              //   0 -  -50 => 4
              // -51 -  -70 => 3
              // -71 -  -90 => 2
              // -91 - -100 => 1
              //xSemaphoreTake( mutexWS2, portMAX_DELAY );
                websocket2.broadcastTXT (jsonWIFI.c_str(), jsonWIFI.length());
              //xSemaphoreGive( mutexWS2 );
              newWIFI = false;
          }
          
          if (newLIPO) {
              //xSemaphoreTake( mutexWS2, portMAX_DELAY );
                websocket2.broadcastTXT (jsonLIPO.c_str(), jsonLIPO.length());
              //xSemaphoreGive( mutexWS2 );
              newLIPO = false;
          }

          if (newADJUST) {
              jsonADJUST = "{\"ADJUST\":{";
              jsonADJUST += "\"pitch\":"; jsonADJUST += String(-1.0*pitch_calibrado_temp) + ",";
              jsonADJUST += "\"roll\":"; jsonADJUST += String(-1.0*roll_calibrado_temp);
              jsonADJUST += "}}";
              //xSemaphoreTake( mutexWS2, portMAX_DELAY );
                websocket2.broadcastTXT (jsonADJUST.c_str(), jsonADJUST.length());
              //xSemaphoreGive( mutexWS2 );
              newADJUST = false;     
          }

          if (newSYNC) {
              jsonSYNC = "{\"ERROR\":\"DESINCRONIZACION > 1%\"}";   
              //xSemaphoreTake( mutexWS2, portMAX_DELAY );
                websocket2.broadcastTXT (jsonSYNC.c_str(), jsonADJUST.length());
              //xSemaphoreGive( mutexWS2 );
              newSYNC = false;
          }
    }

    //********** ENCABEZADO/LEYENDA DE LOS PLOTS **********
    NL = false;

    if (newPlt) {
      if (pltPitch) { Serial.print("PITCH-ang");Serial.print(" ");Serial.print("PITCH-acc");Serial.print(" "); NL = true; }
      if (pltRoll) { Serial.print("ROLL-ang");Serial.print(" ");Serial.print("ROLL-acc");Serial.print(" "); NL = true; }
      if (pltYaw) { Serial.print("YAW-ang");Serial.print(" ");Serial.print("YAW-acc");Serial.print(" "); NL = true; }
      if (pltAltitude) { Serial.print("ALT-gnd");Serial.print(" ");Serial.print("ALT-ref");Serial.print(" "); NL = true; }

      if (plt_ESC_PROA) { Serial.print("PROA");Serial.print(" "); NL = true; }
      if (plt_ESC_POPA) { Serial.print("POPA");Serial.print(" "); NL = true; }
      if (plt_ESC_BABOR) { Serial.print("BABOR");Serial.print(" "); NL = true; }
      if (plt_ESC_ESTRIBOR) { Serial.print("ESTRIBOR");Serial.print(" "); NL = true; }
      if (plt_SERVO) { Serial.print("SERVO");Serial.print(" "); NL = true; }

      if (tiemposBuclePrincipal_250HZ) { 
        Serial.print("ACTIVACION_250HZ");Serial.print(" ");Serial.print("PROCESO_250HZ");Serial.print(" ");Serial.print("MEDIA_250HZ");Serial.print(" "); 
        NL = true; 
      }
      if (tiemposBuclePrincipal_100HZ) { 
        Serial.print("ACTIVACION_100HZ");Serial.print(" ");Serial.print("PROCESO_100HZ");Serial.print(" ");Serial.print("PROCESO_100HZ");Serial.print(" "); 
        NL = true; 
      }
            
      if (NL) Serial.println();
      
      newPlt = false;  
    }

    //********** TEMPORAL DE LOS PLOTS **********
    NL = false;
    
    if (pltPitch==1) { Serial.print(angle_pitch_output);Serial.print(","); Serial.print(angular_motion_pitch);Serial.print(","); NL=true; }
    if (pltRoll==1) { Serial.print(angle_roll_output);Serial.print(","); Serial.print(angular_motion_roll);Serial.print(","); NL=true; }
    //Angulos y velocidades "yaw" sin calibrar por el compas, es decir, los datos son 'crudos'.
    if (pltYaw==1) { Serial.print(angle_yaw_output);Serial.print(","); Serial.print(angular_motion_yaw);Serial.print(","); NL=true; }  
    if(pltAltitude==1) { Serial.print(hGND);Serial.print(",");Serial.print(hREF);Serial.print(","); NL=true; }

    if (plt_ESC_PROA==1) { Serial.print(prop.pwmPROA);Serial.print(","); NL=true; }
    if (plt_ESC_POPA==1) { Serial.print(prop.pwmPOPA);Serial.print(","); NL=true; }
    if (plt_ESC_BABOR==1) {Serial.print(prop.pwmBABOR);Serial.print(","); NL=true;}
    if (plt_ESC_ESTRIBOR==1) {Serial.print(prop.pwmESTRIBOR);Serial.print(","); NL=true;}    
    if (plt_SERVO==1) {Serial.print(prop.anguloSERVO);Serial.print(","); NL=true;}

    if (tiemposBuclePrincipal_250HZ) { 
      Serial.print(t1_250HZ);Serial.print(",");Serial.print(t2_250HZ);Serial.print(",");Serial.print(MEDIA_250HZ);Serial.print(",");
      NL=true; 
    }
    if (tiemposBuclePrincipal_100HZ) { 
      Serial.print(t1_100HZ);Serial.print(",");Serial.print(t2_100HZ);Serial.print(",");Serial.print(MEDIA_100HZ);Serial.print(",");
      NL=true; 
    }
    
    if (NL) Serial.println();

    vTaskDelay(retardo);
  }
  
}



//*****************************************************************
//************************** TAREA NRF24 **************************
//*****************************************************************
void loop6 (void *parameter) {
  TickType_t retardo = pdMS_TO_TICKS(200);

  Serial.print("Tarea NRF24 \n");

  for (;;) {
    vTaskDelay(retardo);
  }    
   
}



//********************************************************************
//************************** TAREA LoopTask **************************
//********************************************************************
void loop0 (void *parameter) {
  TickType_t retardo = pdMS_TO_TICKS(25);
  
  //
  //En el fichero de configuracion .../esp32/1.04/tools/sdk/include/config/sdkconfig.h, el parametro #define CONFIG_FREERTOS_HZ 
  //esta puesto a un valor de 1000 y por tanto, todos los valores de retardo o ticks de tareas se expresaran en milisegundos.
  //

  //https://github.com/espressif/esp-idf/blob/master/components/freertos/Kconfig
  //https://docs.espressif.com/projects/esp-idf/en/v3.3.1/api-reference/system/wdts.html
  //https://docs.espressif.com/projects/esp-idf/en/v3.3.1/api-reference/kconfig.html

  Serial.print("Tarea LoopTask (COM) \n");

  for (;;) {
    websocket1.loop();
    websocket2.loop();
    
    server.handleClient();

    vTaskDelay(retardo); //taskYIELD();
  }   
  
}



//*****************************************************************************
//*********************************** SETUP ***********************************
//*****************************************************************************
void setup() {
    Serial.begin(115200);
    while (!Serial) { ; } //¡¡¡ OJO, quitar esta sentencia cuando el sistema este en produccion !!!

    //******************** OPCION: STATION ********************
    Serial.println("");
    Serial.println("********** WIFI STATION **********");
    Serial.print("Connecting to "); Serial.print(ssid); Serial.print(" "); Serial.println(password);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
    }
    Serial.println("");
    Serial.println("WiFi connected.");
    Serial.print("IP address: "); Serial.println(WiFi.localIP());
    Serial.print("RSSI: "); Serial.println(WiFi.RSSI());
    Serial.println("");
    //******************** OPCION: STATION ********************

    //******************** OPCION: ACCESS POINT ********************
    /*
    //Nos conectaremos a la wifi "MultiRotorFlex" (sin password), y a continuacion a la "10.0.0.1" via browser.
    Serial.println("********** WIFI ACCESS POINT **********");
    WiFi.mode(WIFI_AP);
    WiFi.softAP(ssid2, password2);
    WiFi.softAPConfig(ip, gateway, subnet);
    */
    //******************** OPCION: ACCESS POINT ********************

    server.begin();
    server.on("/", handleRoot); //PC
    server.on("/m", handleRoot2); //moviles
    server.on("/M", handleRoot2); //moviles

    websocket1.begin();
    websocket1.onEvent(WebSocketEvent1);
    websocket2.begin();
    websocket2.onEvent(WebSocketEvent2);

    mutexI2C = xSemaphoreCreateMutex(); //Semaforo paea el acceso al bus I2C
    mutexWS1 = xSemaphoreCreateMutex(); //Semaforo para el acceso a datos WebSockets1
    mutexWS2 = xSemaphoreCreateMutex(); //Semaforo para el acceso a datos WebSockets2
    mutexPOS = xSemaphoreCreateMutex(); //Semaforo para el acceso a la cola de posiciones
    
    binario = xSemaphoreCreateBinary();

    init_propulsion_ledc(); //*** IMPORTANTE ***

    xTaskCreatePinnedToCore( //MPU6050 + HMC5883L
      loop1, //la funcion que contiene mi nucleo infinito
      "SENSORES_250HZ", //a la que le doy un nombre (vale cualquier nombre)
      10000, //el tamaño de la pila
      NULL, //parametros que le quiera pasar al loop
      10, //configMAX_PRIORITIES - 1, //PRIORIDAD MAXIMA = configMAX_PRIORITIES – 1
      &Task1, //nombre "verdadero" de la tarea
      1 //NUCLEO DE SERVICIOS ASIGNADO 
    );
    
    while (!sensoresCalibrados_250HZ) {
      vTaskDelay(100);
    }

    xTaskCreatePinnedToCore( //MS5611 o BMP180
      loop2, //la funcion que contiene mi nucleo infinito
      "SENSORES_100HZ", //a la que le doy un nombre (vale cualquier nombre)
      10000, //el tamaño de la pila
      NULL, //parametros que le quiera pasar al loop
      5, //configMAX_PRIORITIES - 1, //PRIORIDAD MAXIMA = configMAX_PRIORITIES – 1
      &Task2, //nombre "verdadero" de la tarea
      1 //NUCLEO PRINCIPAL ASIGNADO 
    );
        
    while (!sensoresCalibrados_100HZ) {
      vTaskDelay(100);
    }

    if (sensoresCalibrados_250HZ && sensoresCalibrados_100HZ) {
      Serial.print("Lanzando el resto de las tareas en el nucleo ");
      Serial.println("( "+String(xPortGetCoreID())+" ) ... ");
    }

    xTaskCreatePinnedToCore( //GPS
        loop3, "GPS", 2000, NULL, 1, &Task3, 0
    );
    vTaskDelay(100);

    xTaskCreatePinnedToCore( //Bateria, WiFi, ...
        loop4, "MISC", 2000, NULL, 1, &Task4, 0
    );
    vTaskDelay(100);

    xTaskCreatePinnedToCore( //MENSAJERIA
        loop5, "MENSAJERIA", 5000, NULL, 2, &Task5, 0
    );

    xTaskCreatePinnedToCore( //NRF24
        loop6, "NRF24", 2000, NULL, 2, &Task6, 0
    );
    vTaskDelay(100);

    xTaskCreatePinnedToCore( //LoopTask (COM)
        loop0, //la funcion que contiene mi nucleo infinito
        "LoopTask", //a la que le doy un nombre (vale cualquier nombre)
        10000, //el tamaño de la pila
        NULL, //parametros que le quiera pasar al loop
        5, //PRIORIDAD = 5
        &Task0, //nombre "verdadero" de la tarea
        0 //NUCLEO DE SERVICIOS ASIGNADO 
    );

    vTaskDelay(100);

    /*
    //Tareas propias del ESP32
    //------------------------
    Task Name   Task #  Priority  Stack   CPU    
    esp_timer    1      22        4180    0
    Tmr Svc      8       1        1468    0
    ipc0         2      24         604    0
    IDLE0        6       0         396    0

    loopTask    12       1        5188    1
    ipc1         3      24         480    1
    IDLE1        7       0         592    1
    */
}



//****************************************************************************
//*********************************** LOOP ***********************************
//****************************************************************************
void loop() {
    //*** loop()=¿loopTask()? esta por defecto asignada al "CORE1" y con una PRIORIDAD=1 ***  
    //Serial.println("loop()=>\t"+"nucleo:"+String(xPortGetCoreID())+" "+
    //                            "prioridad:"+String(uxTaskPriorityGet(NULL))+" "+
    //                            "stack:"+String(uxTaskGetStackHighWaterMark(NULL)));

    //websocket1.loop();
    //websocket2.loop();
    
    //server.handleClient();

    vTaskDelete(NULL); //¡otra solucion para evitar el "Task watchdog got triggered."¡
}



void handleRoot() {
  server.send(200, "text/html", HTML);
}

void handleRoot2() {
  server.send(200, "text/html", HTML2);
}

void WebSocketEvent1 (uint8_t num, WStype_t type, uint8_t *payload, size_t longitud) {
    //Este puerto se utiliza para las "basica" de MANDO y CONTROL (recibe y emite)
    switch(type) {
      case WStype_CONNECTED:{
          IPAddress ip = websocket1.remoteIP(num);
          Serial.printf("[%u] WS1 conectado a la URL remota : %d.%d.%d.%d - %d \n", num, ip[0], ip[1], ip[2], ip[3], millis());
          WS1conectado = true;
          //Solo mandaremos el mensaje "vorwaerts" por el WS1
          String vorwaerts = "{\"vorwaerts\":true}";
          //xSemaphoreTake( mutexWS1, portMAX_DELAY );
            websocket1.sendTXT(num, vorwaerts);
          //xSemaphoreGive( mutexWS1 );
      }
      break;
      case WStype_DISCONNECTED:{
          Serial.printf("[%u] WS1 desconectado! - %d \n", num, millis());
          WS1conectado = false;
          //if (status == ENAIRE) EMERGENCIAS();
      }
      break;
      case WStype_TEXT:{
          //Serial.printf("[%u] - WS1 recibido: %s \n", num, (char*)payload);
          tiposMensaje((char*)payload);
      }
      break;
      case WStype_BIN:
      case WStype_ERROR:
      case WStype_FRAGMENT_TEXT_START:
      case WStype_FRAGMENT_BIN_START:
      case WStype_FRAGMENT:
      case WStype_FRAGMENT_FIN:
      default:
        break;
    }
}

void WebSocketEvent2 (uint8_t num, WStype_t type, uint8_t *payload, size_t longitud) {
    //Este puerto se utiliza en exclusiva para transmitir "info" de sensores (emite y no recibe)
    switch(type) {
      case WStype_CONNECTED:{
          IPAddress ip = websocket2.remoteIP(num);
          Serial.printf("[%u] WS2 conectado a la URL remota : %d.%d.%d.%d - %d \n", num, ip[0], ip[1], ip[2], ip[3], millis());
          WS2conectado = true;
          //El manesaje de conexion "vorwaerts" solo se manda por el WS1
      }
      break;
      case WStype_DISCONNECTED:{
          Serial.printf("[%u] WS2 desconectado! - %d \n", num, millis());
          WS2conectado = false;
          //if (status == ENAIRE) EMERGENCIAS();
      }
      break;
      case WStype_TEXT:
      case WStype_BIN:
      case WStype_ERROR:
      case WStype_FRAGMENT_TEXT_START:
      case WStype_FRAGMENT_BIN_START:
      case WStype_FRAGMENT:
      case WStype_FRAGMENT_FIN:
      default:
        break;
    }
}



//************************** MENSAJERIA **************************
void tiposMensaje(String str) {
  int pos;
  String cmd, dev;
  int P1, P2, P3, P4, P5;

  pos = str.indexOf(":"); //encontrar el limitador ":" empezando por la posicion "cero" del string
  dev = str.substring(0,pos); //desde el principio del string hasta la posicion del limitador
  cmd = str.substring(pos+1); //desde el (limitador+1) hasta el final del string
  //Serial.print("dev:"); Serial.println(dev);
  //Serial.print("cmd:"); Serial.println(cmd);

  // *********************************** MANDOS DE VUELO ***********************************
  if(dev == "THROTTLE") {
          testing = false;
          const char * strings = cmd.c_str(); //para descomponer "cmd" en parametros separados por comas
          sscanf(strings, "%d", &P1);
          //Serial.print("THROTTLE:");Serial.println(P1);
          prop.throttle = (unsigned int)P1;

          if (prop.throttle >= (prop.OFFSET+prop.UMBRAL))
            volando = true;
          else
            volando = false;

          if ((prop.throttle >= (prop.OFFSET+prop.UMBRAL)) && (status == PARADO)) {
            started = true; //En la transicion PARADO/ENAIRE reseteamos sensores
            status = ENAIRE;
            return;
          }
          
          if ((prop.throttle < (prop.OFFSET+prop.UMBRAL)) && (status == ENAIRE)) {
            //started = true; //En la transicion ENAIRE/PARADO reseteamos sensores
            status = PARADO;
            return;
          }
          
          return;
  }

  if (dev == "JS") {
          testing = false;
          const char * strings = cmd.c_str(); //para descomponer "cmd" en parametros separados por comas
          sscanf(strings, "%d,%d", &P1, &P2);
          //Serial.print("JS:");Serial.print(P1);Serial.print(",");Serial.println(P2);
          prop.joystickX = P1; // criterio web.h => derecha=positivo y izquierda=negativo
          prop.joystickY = P2; // criterio web.h => arriba=negativo y abajo=positivo

          //if ((prop.joystickX == 0) && (prop.joystickY == 0))
          //  maniobrando = false;
          //else
          //  maniobrando = true;
                                
          return;
  }

  // *********************************** TEST Y AJUSTES EN TIERRA ***********************************
  if (dev == "testPROA") {
      if (!volando) {
        //Serial.println("testPROA");   
        testing = true;
        vTaskDelay (10); //2,5ciclos de espera
        testPROA();
        testing = false;
      }
      return;
  }
  
  if (dev == "testPOPA") {
      if (!volando) {
        //Serial.println("testPOPA"); 
        testing = true;
        vTaskDelay (10); //2,5ciclos de espera
        testPOPA();
        testing = false;
      }
      return;
  }

  if (dev == "testESTRIBOR") {
      if (!volando) {
        //Serial.println("testESTRIBOR");   
        testing = true;
        vTaskDelay (10); //2,5ciclos de espera
        testESTRIBOR();
        testing = false;
      }
      return;
  }

  if (dev == "testBABOR") {
      if (!volando) {
        //Serial.println("testBABOR");   
        testing = true;
        vTaskDelay (10); //2,5ciclos de espera
        testBABOR();
        testing = false;
      }
      return;
  }

  if (dev == "testSERVO") {
      if (!volando) {
        //Serial.println("testSERVO");   
        testing = true;
        vTaskDelay (10); //2,5ciclos de espera
        testSERVO();
        testing = false;
      }
      return;
  }

  if (dev == "propulsion") {
      if (!volando) {
        testing = true;
        const char * strings = cmd.c_str(); //para descomponer "cmd" en parametros separados por comas
        sscanf(strings, "%d,%d,%d,%d,%d", &P1, &P2, &P3, &P4, &P5);
        //Serial.print("propulsion:");
        //Serial.print(P1);Serial.print(','); Serial.print(P2);Serial.print(','); Serial.print(P3);Serial.print(','); Serial.print(P4);Serial.print('\t'); Serial.println(P5);
        vTaskDelay (10); //2,5ciclos de espera
        setTestProp(P1, P2, P3, P4, P5);
        //testing = false; //Los tests de PROA, POPA, BABOR y ESTRIBOR son limitados en el tiempo; estos test de propulsion NO lo son.
      } 
      return;
  }

  if (dev == "SYNCmotor") { //*** IMPORTANTE ***
      if (!volando) {
        //Serial.println("SYNCmotor");
        testing = true;
        vTaskDelay (10); //2,5ciclos de espera
        init_propulsion_ledc (); //!!! OJO !!!
        testing = false;
      }
      return;
  }

  // *********************************** PLOTTING ***********************************
  if (dev == "trazas") {
    //Serial.println(cmd);
    const char * strings = cmd.c_str();
    sscanf(strings, "%i,%i,%i,%i,   %i,%i,%i,%i,%i,   %i,%i,   %i,%i", 
                     &pltPitch, &pltRoll, &pltYaw, &pltAltitude, 
                     &plt_ESC_PROA, &plt_ESC_POPA, &plt_ESC_BABOR, &plt_ESC_ESTRIBOR, &plt_SERVO,
                     &FiltroFusionSensoresPRE, &FiltroFusionSensoresPOS,
                     &tiemposBuclePrincipal_250HZ, &tiemposBuclePrincipal_100HZ);
    //Serial.println("----------");
    //Serial.printf ("Pitch:%i, Roll:%i, Yaw:%i, Altitude:%i \n", pltPitch, pltRoll, pltYaw, pltAltitude);
    //Serial.printf ("ESC_PROA:%i, ESC_POPA:%i, ESC_BABOR:%i, ESC_ESTRIBOR:%i, SERVO:%i \n", plt_ESC_PROA, plt_ESC_POPA, plt_ESC_BABOR, plt_ESC_ESTRIBOR, plt_SERVO);
    //Serial.printf ("Fusion de sensores PRE:%i \n", FiltroFusionSensoresPRE);
    //Serial.printf ("Fusion de sensores POS:%i \n", FiltroFusionSensoresPOS);
    //Serial.printf ("Tiempos de proceso del bucle 250HZ:%i \n", tiemposBuclePrincipal_250HZ);
    //Serial.printf ("Tiempos de proceso del bucle 100HZ:%i \n", tiemposBuclePrincipal_100HZ);
    //Serial.println("----------");
    newPlt = true;          
    return;
  }  

  // *********************************** AJUSTES **************************/*********
  if (dev == "parametros") {
    //Serial.println(cmd);
    if (!volando) {
      testing = true;
      const char * strings = cmd.c_str();
      sscanf(strings,"%i,%f,%f,%f,%f,%i,  %i,%f,%f,%f,%f,%i,  %f,%f,%f,%i,  %f,%f,%f,%f,%i",
                      &SetpointPitch, &KPpitch,       &KIpitch,    &KDpitch,    &aggKPpitch,    &aggPitchAngle,
                      &SetpointRoll,  &KProll,        &KIroll,     &KDroll,     &aggKProll,     &aggRollAngle,
                      &KPyaw,         &KIyaw,         &KDyaw,      &DRCG,
                      &KPASCaltitude, &KPDESaltitude, &KIaltitude, &KDaltitude, &AltitudeLimits);
      //Serial.println("----------");
      //Serial.printf("SPpitch:%i KPpitch:%f KIpitch:%f KDpitch:%f aggKPpitch:%f aggPitchAngle:%i \n", SetpointPitch, KPpitch, KIpitch, KDpitch, aggKPpitch, aggPitchAngle);
      //Serial.printf("SProll:%i  KProll:%f  KIroll:%f  KDroll:%f  aggKProll:%f  aggRollAngle:%i \n",  SetpointRoll,  KProll,  KIroll,  KDroll,  aggKProll,  aggRollAngle); 
      //Serial.printf("KPyaw:%f KIyaw:%f KDyaw:%f DRCG:%i \n", KPyaw, KIyaw, KDyaw, DRCG);
      //Serial.printf("KPASCaltitude:%f KPDESaltitude:%f KIaltitude:%f KDaltitude:%f AltitudeLimits:%i \n", KPASCaltitude, KPDESaltitude, KIaltitude, KDaltitude, AltitudeLimits);
      //Serial.println("----------");
      setPIDtunings();
    }
    return;
  }

  if (dev == "drift") {
      //Serial.println(cmd);
      if (!volando) {
        testing = true;
        const char * strings = cmd.c_str();
        sscanf(strings, 
               "%f,%f,%f,%f,%f,%f,   %f,%d,%f,%f,%f,%f,   %f,%f",
               &DRIFT_PITCH_ROLL, &DRIFT_YAW, &PITCH_GAIN, &ROLL_GAIN, &YAW_GAIN, &SENSIBILITY,   
               &FILTERpos, &DESVIACION_PRESION_PERMITIDA, &DESVIACION_1G_1M, &DESVIACION_1G_MAX, &ATENUACION_FILTRO, &AMPLIFICACION_FILTRO,
               &FLL, &PARACHUTE);
        //Serial.println("----------");
        //Serial.printf("Sensor Fusion (pitch & roll): %f \n", DRIFT_PITCH_ROLL);
        //Serial.printf("Sensor Fusion (yaw): %f \n", DRIFT_YAW);
        //Serial.printf("PITCH GAIN (accelerometer): %f \n", PITCH_GAIN);
        //Serial.printf("ROLL GAIN (accelerometer): %f \n", ROLL_GAIN);
        //Serial.printf("YAW GAIN (accelerometer): %f \n", YAW_GAIN);
        //Serial.printf("SENSIBILITY (gyro): %f \n", SENSIBILITY);
        //Serial.print("\n");
        //Serial.printf("ALTITUDE FINE FILTER: %f \n", FILTERpos);
        //Serial.printf("MAXIMUN ALLOWED PEAKS (pascal): %d \n", DESVIACION_PRESION_PERMITIDA);
        //Serial.printf("GRAVITY (1G = 9,81 m/s) DRIFT: %f \n", DESVIACION_1G_1M);
        //Serial.printf("MAXIMUN DRIFT: %f \n", DESVIACION_1G_MAX);
        //Serial.printf("ATTENUATION: %f \n", ATENUACION_FILTRO);
        //Serial.printf("AMPLIFICATION: %f \n", AMPLIFICACION_FILTRO);
        //Serial.print("\n");
        //Serial.printf("FLIGHT LEVEL LOGIC (0.1 meter = 0.116 pascal): %f \n", FLL);
        //Serial.printf("PARACHUTE (T/P correction per hour): %f \n", PARACHUTE);
        //Serial.println("----------");
      } 
      return;
  }

  if (dev == "hover") {
    //Serial.println(cmd);
    if (!volando) {
        testing = true;
        const char * strings = cmd.c_str();
        sscanf(strings, "%i", &HOVER);
        //Serial.printf("HOVER with GROUND-EFFECT (PWMs): %i \n", HOVER);
    }
    return;
  }
  
  if (dev == "RESET") {
    if (!volando) {
      started = true;
      //Serial.println("RESET");      
    }
    return;
  }

  if (dev == "ADJUST") {
    if (!volando) {
      autoCalibracion_1();
      adjusted = false;
      //Serial.println("ADJUST");      
    }
    return;
  }

  if (dev == "GRAVITY") {
      if (!volando) {
        testing = true;
        const char * strings = cmd.c_str();
        sscanf(strings, "%f,%f", &pitch_calibrado, &roll_calibrado);
        //Serial.printf("gravedad:%f, %f \n", pitch_calibrado, roll_calibrado);
      } 
      return;
  }

  // *********************************** COMANDOS ***********************************
  if (dev == "CUTOFF") { 
    testing = true; //aunque este el drone "volando" 
    //Serial.println("CUTOFF");
    vTaskDelay (10); //2,5ciclos de espera
    setTestProp(1000, 1000, 1000, 1000, prop.ALIGN_SERVO);
    //testing = false; //Los tests de PROA, POPA, BABOR y ESTRIBOR son limitados en el tiempo; estos test de propulsion NO lo son.
    return;
  }

  if (dev == "ARMED") { 
    testing = true; //aunque este el drone "volando" 
    //Serial.println("ARMED");
    vTaskDelay (10); //2,5ciclos de espera
    setTestProp(prop.UMBRAL, prop.UMBRAL, prop.UMBRAL, prop.UMBRAL, prop.ALIGN_SERVO);
    //testing = false; //Los tests de PROA, POPA, BABOR y ESTRIBOR son limitados en el tiempo; estos test de propulsion NO lo son.
    return;
  }

  if (dev == "swingHOVER")  {
      //Serial.print(dev);Serial.print(":");Serial.println(cmd);
      if ((cmd == "1") && (volando == true)) {
        prop.throttle = HOVER;
        //Serial.println("=> HOVER");
      } else {
        //El mensaje del tipo "swingHOVER:0" no se tiene en cuenta, ya que cualquier accion sobre el "throttle" la invalidad (vuelta a la normalidad).
      }
      return;
  }

  if (dev == "swingHOLD")  {
    //Serial.print(dev);Serial.print(":");Serial.println(cmd);
    if (volando) {
      const char * strings = cmd.c_str(); //para descomponer "cmd" en parametros separados por comas
      sscanf(strings, "%i", &HOLD);
      //Serial.print("HOLD:");Serial.println(HOLD);
      if (HOLD != 0) //implementar la logica de control para el mantenimiento de altitudes
        holding = true;
      else
        holding = false;
    }  
    return;
  }

  if (dev == "swingYAW")  {
    //Serial.print(dev);Serial.print(":");Serial.println(cmd);
      if (cmd == "1") {
        yawing = true;
        //Serial.println("YAWING = true");
      } else {
        yawing = false;
        //Serial.println("YAWING = false");
      }
    return;
  }

}
//************************** MENSAJERIA **************************



void EMERGENCIAS () {}
