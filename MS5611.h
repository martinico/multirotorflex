
#ifndef MS5611_H
#define MS5611_H

  //Constantes y variables declaradas en este modulo y exportables al MAIN
  extern float hGND, hREF, hQNH;
  
  extern int PIDaltimetro;

  extern float FILTERpos;
  extern int64_t DESVIACION_PRESION_PERMITIDA;
  extern float DESVIACION_1G_1M;
  extern float DESVIACION_1G_MAX;
  extern float ATENUACION_FILTRO;    
  extern float AMPLIFICACION_FILTRO;    

  extern int FiltroFusionSensoresPRE, FiltroFusionSensoresPOS;

  //metodos globales
  void init_MS5611();  
  bool calculateAltitude();
  void resetAltitude();
  float convertAltitude(float P);
  float filterAltitude(float P);
  int parachutePID();

  //metodos locales
  void start_MS5611();  
  int reset_MS5611();

  void requestTemperature();
  void getTemperature();
  void calcTemperature();
  
  void requestPressure();
  void getPressure();
  void calcPressure();
  void calcParachute();
 
#endif

/*
ESPECIFICACIONES:
    High resolution module, 10 cm
    Resolution (RMS), 0.012 mbar => 0.012 * 0.117 = 0.001404 metros
    Fast conversion down to 1 ms
    Low power, 1 µA (standby < 0.15 µA)
    Supply voltage 1.8 to 3.6 V
    Integrated digital pressure sensor (24 bit ΔΣ ADC)
    Operating range: 10 to 1200 mbar, -40 to +85 °C
    I2C and SPI interface up to 20 MHz
    No external components (Internal oscillator)
    MAX.PRECISSION
      min=7.40 typical=8.22 max=9.04 (ms)
      Convert D1 (OSR=4096) =  0x48
      Convert D2 (OSR=4096) =  0x58
*/

/*
The barometric pressure setting on the altimeter is usually one of three possibilities: 
- QFE is the pressure setting at your airfield which results in the height display showing above the field (Field Elevation).
- QNH is the current atmospheric pressure at sea level (Nautical Height), so gives the height above sea level.
- QNE is 1013.25mb, and is used when following Flight Levels (FL) as opposed to height (Cat A airspace). 
  This is how vertical separation is assured - all aircraft in the airway use the QNE setting, so if the atmospheric pressure drops they all fly a bit lower, but maintain the gaps.
*/
