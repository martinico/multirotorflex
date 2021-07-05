
#ifndef FILTROC_H
#define FILTROC_H

  class FiltroComplementario {

  public:
    FiltroComplementario(float error_maximo, float desviacion_maxima, float G); //constructor
    
    float nuevaEstimacion(float medida);
    
    void setErrorMaximo(float error_maximo);
    void setDesviacionMaxima(float desviacion_maxima);
    void setGanancia(float G);
  
  private:
    float _H_actual;
    float _H_rapida;
    float _H_lenta;
    float _H_diferencia;
    
    float _error_maximo;
    float _desviacion_maxima;
    float _G;
  };

#endif

