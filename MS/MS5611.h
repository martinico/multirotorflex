
#ifndef MS5611_H
#define MS5611_H

  //Constantes y variables declaradas en este modulo y exportables al MAIN
  extern const float METROS_A_MILIBARES;
  extern const float PRESION_STANDARD;
  extern float hGND, hREF;

  //Constantes y variables declaradas en el MAIN y exportables a cualquier modulo
  extern const uint16_t FREQUENCY;
  extern const uint16_t SAMPLETIME;
  extern int PIDaltimetro;

  //metodos globales
  void initMS5611();  
  bool calculateAltitude();
  float convertAltitude(float P);
  void resetAltitude();

  //metodos locales
  void startMS5611();  
  int resetMS5611();
  void requestTemperature();
  void getTemperature();
  void requestPressure();
  void getPressure();
  
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
