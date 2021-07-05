
#ifndef BMP180_H
#define BMP180_H

  //Constantes y variables declaradas en este modulo y exportables al MAIN
  extern float hGND, hREF, hQNH;
  //The barometric pressure setting on the altimeter is usually one of three possibilities: 
  //QFE : is the pressure setting at your airfield which results in the height display showing above the field (Field Elevation).
  //QNH : is the current atmospheric pressure at sea level (Nautical Height), so gives the height above sea level.
  //QNE : is 1013.25mb, and is used when following Flight Levels (FL) as opposed to height (Cat A airspace). 
  //      This is how vertical separation is assured - all aircraft in the airway use the QNE setting, 
  //      so if the atmospheric pressure drops they all fly a bit lower, but maintain the gaps.
  
  extern int PIDaltimetro;

  void init_BMP180 ();
  void reset_BMP180();

  // METODOS CLASICOS SIMILARES A LOS IMPLEMENTADOS PARA EL MS5611 (14msec)
  void calcularAltitudes();
  void filtroSimple();
  void filtroDoble();
  void filtroFuzzy();
  void parachute();
  int PIDparachute();
  void trazas();
  
  void pedirTemperatura();
  void leerTemperatura();
  void calcularTemperatura();
  void pedirPresion();
  void leerPresion();
  void calcularPresion();    

  extern int FiltroFusionSensoresPRE, FiltroFusionSensoresPOS;
  extern float FILTERpos;
  extern int64_t DESVIACION_PRESION_PERMITIDA;
  extern float DESVIACION_1G_1M;
  extern float DESVIACION_1G_MAX;
  extern float ATENUACION_FILTRO;    
  extern float AMPLIFICACION_FILTRO;  

#endif

/*
BOSCH-BMP180 es un sensor de alta precisión y baja potencia. 
El rango de medición es de 300hPa a 1110 hPa, equivalente a una altitud de -500m a 9000m sobre el nivel del mar. 
La precisión es configurable, desde 0.06hPa (0.5 metros) en el modo de bajo consumo, a 0.02hPa (0.17 metros) en el modo de alta precisión.
*/
