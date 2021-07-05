
#ifndef PID_H
#define PID_H

  extern int SetpointPitch;
  extern float KPpitch;
  extern float KIpitch;
  extern float KDpitch;
  extern float aggKPpitch;
  extern int aggPitchAngle;

  extern int SetpointRoll;
  extern float KProll;
  extern float KIroll;
  extern float KDroll;
  extern float aggKProll;
  extern int aggRollAngle;

  extern int SetpointYaw;
  extern float KPyaw;
  extern float KIyaw;
  extern float KDyaw;
  extern int DRCG;

  extern float SetpointAltitude;
  extern float KPASCaltitude;
  extern float KPDESaltitude;
  extern float KPaltitude;
  extern float KIaltitude;
  extern float KDaltitude;
  extern int AltitudeLimits;
    
  //Constantes declaradas en el MAIN y exportables a cualquier modulo
  extern const uint16_t FREQUENCY;
  extern const uint16_t SAMPLETIME;

  //Variables para el control PID externo
  extern float PITCH_GAIN;
  extern float ROLL_GAIN;
  extern float YAW_GAIN;
  extern float SENSIBILITY;
  
  //metodos YMFC
  float calculatePitchPID (int referencia, float angulo, float velocidad);
  float calculateRollPID (int referencia, float angulo, float velocidad);
  float calculateYawPID (int referencia, float angulo, float velocidad);
  
  //metodos clasicos
  float calculate2PitchPID (int referencia, float angulo);
  float calculate2RollPID (int referencia, float angulo);
  float calculate2YAWMotionPID (int referencia, float velocidad);
  float calculate2YAWPositionPID (int referencia, float angulo);
  
  //metodos modernos
  float calculate3PitchPID (int referencia, float angulo);
  float calculate3RollPID (int referencia, float angulo);
  float calculate3YawPID (int referencia, float angulo);
  float calculate3AltitudePID (float referencia, float altitude);
  
  //funciones especiales
  void setPIDtunings ();
  void resetPIDerrors ();

  float calculateOnlyGyroPID (int referencia);
#endif
