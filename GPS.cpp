
#include <Arduino.h>
#include "GPS.h";
#include <TinyGPS++.h>

#define TRAZAS_GPSNEO 0

TinyGPSPlus gps;

TinyGPSCustom vdop(gps, "GPGSA", 17); // $GPGSA sentence, 17th element (VDOP)
TinyGPSCustom decV(gps, "GPRMC", 10); // $GPRMC sentence, 10th element - Magnetic Declination Value
TinyGPSCustom decO(gps, "GPRMC", 11); // $GPRMC sentence, 11th element - Magnetic Declination East/West
//TinyGPSCustom geod(gps, "GPGCA", 11); // $GPGGA sentence, 11th element - Geoidal Separation - difference between WGS-84 earth ellipsoid and mean sea level (negative -> geoid is below WGS-84 ellipsoid)

#define RXD2 16
#define TXD2 17
const int baudiosGPS = 9600;

struct datosGPSNEO datosGPS;

void startGPSNEO () {
  Serial2.begin(baudiosGPS, SERIAL_8N1, RXD2, TXD2);
}

bool testGPSNEO (unsigned long ms) {
    unsigned long start = millis();
    do {
      while (Serial2.available() > 0) {
          gps.encode(Serial2.read());
          if (gps.location.isUpdated() &&
              gps.altitude.isUpdated() && 
              gps.date.isUpdated() && 
              gps.time.isUpdated() && 
              gps.speed.isUpdated() && 
              gps.course.isUpdated() && 
              gps.hdop.isUpdated() && 
              gps.satellites.isUpdated()) {  
                
                #if TRAZAS_GPSNEO
                  Serial.print(F(" Fecha:")); Serial.println(gps.date.value());
                  Serial.print(F(" Hora:")); Serial.println(gps.time.value());
                  Serial.print(F(" Latitud:")); Serial.println(gps.location.lat());
                  Serial.print(F(" Longitud:")); Serial.println(gps.location.lng());
                  Serial.print(F(" HDOPx100:")); Serial.println(gps.hdop.value());
                  Serial.print(F(" Altura:")); Serial.println(gps.altitude.meters());
                  Serial.print(F(" VDOP:"));
                  if (vdop.isUpdated()) {
                    Serial.println(vdop.value());
                  } else {
                    Serial.println("¿999?");
                  }
                  Serial.print(F(" Velocidad:")); Serial.println(gps.speed.kmph());
                  Serial.print(F(" Curso:")); Serial.println(gps.course.deg());
                  Serial.print(F(" Declinacion:")); Serial.println(declinacion);
                  Serial.print(F(" Satelites:")); Serial.println(gps.satellites.value());
                  Serial.println(F("zczc"));
                  Serial.println(F(""));       
                #endif
                
                datosGPS.fecha = gps.date.value();
                datosGPS.hora= gps.time.value();
                datosGPS.latitud = gps.location.lat();
                datosGPS.longitud = gps.location.lng();
                datosGPS.HDOP = gps.hdop.value();
                datosGPS.altitud = gps.altitude.meters();
                datosGPS.VDOP = 100 * atof(vdop.value()); //invalid conversion from 'const char*' to 'long int' [-fpermissive]
                datosGPS.velocidad = gps.speed.kmph();
                datosGPS.direccion = gps.course.deg();
                datosGPS.declinacion = declinacion;
                datosGPS.satelites = gps.satellites.value();

                return true;
          }
      }
    } while (millis() - start < ms);
    return false;
}

/*
https://circuits4you.com/2018/12/31/esp32-hardware-serial2-example/

There are THREE serial ports on the ESP32 known as U0UXD, U1UXD and U2UXD all work at 3.3V TTL Level. 
There are THREE hardware supported serial interfaces on the ESP32 known as UART0, UART1 and UART2. 
Like all peripherals, the pins for the UARTs can be logically mapped to any of the available pins on the ESP32. 
However, the UARTs can also have direct access which marginally improves performance. 
The pin mapping table for this hardware assistance is as follows:
UART    RX-IO   TX-IO   CTS     RTS
-----------------------------------
UART0   GPI03   GPI01   NA      NA
UART1   GPI09   GPI10   GPIO6   GPI11
UART2   GPI16   GPI17   GPI08   GPI07
- Both UARTs must operate at about the same baud rate. 
- The baud rate between the transmitting and receiving UARTs can only differ by about 3% before the timing of bits gets too far off. 
- Both UARTs must also must be configured to transmit and receive the same data packet structure.

// There are three serial ports on the ESP known as U0UXD, U1UXD and U2UXD.
// U0UXD is used to communicate with the ESP32 for programming and during reset/boot.
// U1UXD is unused and can be used for your projects. Some boards use this port for SPI Flash access though
// U2UXD is unused and can be used for your projects.
 
#define RXD2 16
#define TXD2 17
 
void setup() {
  // Note the format for setting a serial port is as follows: Serial2.begin(baud-rate, protocol, RX pin, TX pin);
  Serial.begin(115200);
  //Serial1.begin(9600, SERIAL_8N1, RXD2, TXD2);
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);
  Serial.println("Serial Txd is on pin: "+String(TX));
  Serial.println("Serial Rxd is on pin: "+String(RX));
}
 
void loop() { //Choose Serial1 or Serial2 as required
  while (Serial2.available()) {
    Serial.print(char(Serial2.read()));
  }
*/
