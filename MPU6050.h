
#ifndef MPU6050_H
#define MPU6050_H

  extern float angle_yaw; //yaw "crudo"
  
  extern float angle_pitch_output, angle_roll_output, angle_yaw_output;
  extern float angular_motion_pitch, angular_motion_roll, angular_motion_yaw;

  extern float DRIFT;
  extern float DRIFT_PITCH;
  extern float DRIFT_ROLL;
  extern float DRIFT_PITCH_ROLL;
  extern float DRIFT_YAW;
  
  //extern float HPF; //HIGH-PASS FILTER to dampen the pitch & roll angles
  //extern float LPF; //LOW-PASS FILTER to cutoff 10Hz frequency in the motion

  extern float pitch_calibrado, roll_calibrado;
  extern float pitch_calibrado_temp, roll_calibrado_temp;

  extern int FiltroFusionSensores;
  
  //Constantes declaradas en el MAIN y exportables a cualquier modulo
  extern const uint16_t FREQUENCY;
  extern const uint16_t SAMPLETIME;  
  
  void init_MPU6050();
  void setup_MPU6050_registers();
  void read_MPU6050_data();
  void calibration_MPU6050();

  void calculateAnglesMotion();
  void resetGyroAngles(); 

  void autoCalibracion_1();
  bool autoCalibracion_2();

#endif
