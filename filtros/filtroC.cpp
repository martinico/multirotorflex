
#include <Arduino.h>
#include "filtroC.h"

FiltroComplementario::FiltroComplementario(float error_maximo, float desviacion_maxima, float G) {
  _error_maximo = error_maximo;
  _desviacion_maxima = desviacion_maxima;
  _G = G;
  
  _H_actual = 0.0;
  _H_rapida = 0.0;
  _H_lenta = 0.0;
  _H_diferencia = 0.0;
}

float FiltroComplementario::nuevaEstimacion(float medida) {
  _H_rapida = _H_rapida * (float)0.9 + medida * (float)0.1;
  _H_lenta = _H_lenta * (float)0.99 + medida * (float)0.01;
  
  _H_diferencia= _H_lenta - _H_rapida;

  if (_H_diferencia > + _error_maximo) _H_diferencia = + _error_maximo;
  if (_H_diferencia < - _error_maximo) _H_diferencia = - _error_maximo;
  
  if (_H_diferencia > + _desviacion_maxima || _H_diferencia < - _desviacion_maxima) _H_lenta -= _H_diferencia / _G;

  _H_actual = _H_lenta;

  // _H_actual = ceilf(_H_actual*10.0)/10.0; //conversion a un solo digito decimal
  // _H_actual = floorf(_H_actual*10.0)/10.0; //conversion a un solo digito decimal
  // _H_actual = truncf(_H_actual*10.0)/10.0; //conversion a un solo digito decimal
  // _H_actual = roundf(_H_actual*10.0)/10.0; //conversion a un solo digito decimal
  
  return _H_actual;
}

void FiltroComplementario::setErrorMaximo(float error_maximo) {
  _error_maximo = error_maximo;
}

void FiltroComplementario::setDesviacionMaxima(float desviacion_maxima) {
  _desviacion_maxima = desviacion_maxima;
}

void FiltroComplementario::setGanancia(float G) {
  _G =G;
}
