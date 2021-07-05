
//https://www.youtube.com/watch?v=4Q5kJ96YYZ4
//http://www.ilectureonline.com/search/kalman

//Libreria de un Filtro-Kalman unidimensional.

//e-mea (measuremt uncertainty) o cuanto esperamos que nuestra medida varie (¿1 metro?)
//e_est (estimation uncertainty) se incializa con la misma magnitud de "e-mea", ya que el filtro se encargara de ajustarla
//q (proccess variable) es una variable (0,001 - 1) a ajustar (se recomienda 0,01)

#ifndef KALMAN_H
#define KALMAN_H

  class SimpleKalmanFilter {

  public:
    SimpleKalmanFilter(float mea_e, float est_e, float q); //constructor
    float updateEstimate(float mea);
    void setMeasurementError(float mea_e);
    void setEstimateError(float est_e);
    void setProcessNoise(float q);
    float getKalmanGain();
  
  private:
    float _err_measure;
    float _err_estimate;
    float _q;
    float _current_estimate;
    float _last_estimate;
    float _kalman_gain;

  };

#endif

