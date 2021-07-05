
#include <Arduino.h>
#include "PIDcontrol.h"

int SetpointPitch = 0; //PWM
float KPpitch     = 1.8;
float KIpitch     = 6.5;
float KDpitch     = 0.12;
float aggKPpitch  = 1.0;
int aggPitchAngle = 45;

int SetpointRoll = 0; //PWM
float KProll     = 1.5;
float KIroll     = 4.5;
float KDroll     = 0.16;
float aggKProll  = 1.0;
int aggRollAngle = 45;

int SetpointYaw = 0;  //DEGREES
float KPyaw = 1.1;
float KIyaw = 0.05;
float KDyaw = 0.15;
int DRCG = 5;         //Damped Rate of Change Gain

float SetpointAltitude = 0.0; //METROS
float KPASCaltitude = 15.00;
float KPDESaltitude = 5.00;
float KPaltitude    = 0.00;
float KIaltitude    = 1.00;
float KDaltitude    = 0.00;
int AltitudeLimits  = 25;

float ki_pitch, kd_pitch, ki_roll, kd_roll, ki_yaw, kd_yaw, ki_altitude, kd_altitude; //constantes adaptadas al ciclo clasico

float error_pitch, error_roll, error_yaw, error_altitude;

float error_proporcional, error_integral, error_derivado;

float PIDpitch, PIDroll, PIDyaw, PIDaltitude;

float last_error_pitch = 0.0;
float last_error_roll = 0.0;
float last_error_yaw = 0.0;
float last_error_altitude = 0.0;

float last_angulo_pitch = 0.0;
float last_angulo_roll = 0.0;
float last_angulo_yaw = 0.0;
float last_altitude = 0.0;

float error_integral_pitch = 0.0;
float error_integral_roll = 0.0;
float error_integral_yaw = 0.0;
float error_integral_altitude = 0.0;

int GAIN = 15; //Ganancia por error en el angulo equivalente a un KP
float PITCH_GAIN = 10.0;
float ROLL_GAIN = 10.0;
float YAW_GAIN = 10.0;
//1 grado = 11,111 pwm (ya que (2500-500)pwm son 180 grados) => 250 pwm = 22,5 grados

float SENSIBILITY = 3.0; //Ajuste para obtener para el control angulos en grados
//SENSIBILIDAD_MPU6050 = 3.0; //FS_SEL=1=500º/s / 3 = 166,66dps, y a mayor sensibilidad dividiremos por un numero menor

static const float PASO_A_PASO_ANGULAR = 0.4 * SAMPLETIME / 1000.0; //SERVO(no-load) = 0,15sec / 60º, luego 1 milisec = 0.4º y en total 1.6º
static const float PASO_A_PASO_PWM = PASO_A_PASO_ANGULAR * GAIN; //1 milisec = 0.4*11,111=4,444pwm y en total 1.6*11,111=17,77pwm

const float maxErrorPITCH = 400.0;
const float maxErrorROLL = 400.0;
const float maxErrorYAW = 400.0;
const float maxErrorALTITUDE = 100.0;
const float maxErrorINTEGRAL = 400.0;

#define TRAZAS_PITCH 0
#define TRAZAS_ROLL 0
#define TRAZAS_YAW 0
#define TRAZAS_ALTITUDE 0

#define PLOTTER_PITCH 0
#define PLOTTER_ROLL 0
#define PLOTTER_YAW 0
#define PLOTTER_ALTITUDE 0

void resetPIDerrors () {
  last_error_pitch = last_error_roll = last_error_yaw = last_error_altitude = 0.0;
  last_angulo_pitch = last_angulo_roll = last_angulo_yaw = last_altitude = 0.0;
  error_integral_pitch = error_integral_roll = error_integral_yaw = error_integral_altitude = 0.0;
}

void setPIDtunings () {
  float factor_tiempo = SAMPLETIME/1000000.0; //OJO : tiempo en "segundos" de la FRECUENCIA de 250Hz o del SAMPLETIME de 4000µs = 4ms
  
  ki_pitch = KIpitch * factor_tiempo;
  kd_pitch = KDpitch / factor_tiempo;
  
  ki_roll = KIroll * factor_tiempo;
  kd_roll = KDroll / factor_tiempo;
  
  ki_yaw = KIyaw * factor_tiempo;
  kd_yaw = KDyaw / factor_tiempo;

  ki_altitude = KIaltitude * factor_tiempo;
  kd_altitude = KDaltitude / factor_tiempo;
  
  /*
  Serial.print(F("ki_pitch:"));Serial.print(ki_pitch);Serial.print("\t");
  Serial.print(F("kd_pitch:"));Serial.print(kd_pitch);Serial.print("\n");        
  Serial.print(F("ki_toll:"));Serial.print(ki_roll);Serial.print("\t");
  Serial.print(F("kd_roll:"));Serial.print(kd_roll);Serial.println("\n");
  Serial.print(F("ki_altitude:"));Serial.print(ki_altitude);Serial.print("\t");
  Serial.print(F("kd_altitude:"));Serial.print(kd_altitude);Serial.println("\n");
  */
}



//****************************** METODOS YMFC ******************************
//  ki = KI * factor_tiempo;
//  kd = KD / factor_tiempo;
//  factor_tiempo = SAMPLETIME/1000000.0;

//Para una GANANCIA = 15 , SENSIBILIDAD = 3 y KP = 1,3 , los parametros serian:
//KIaplicado = 0,04 => KIteorico = 0.04 * 1000 / 4 = 10
//KDaplicado = 18   => KDteorico = 18 * 4 / 1000 = 0,072
//KPyaw  = 4.0
//KIaplicado = 0.02 => KIteorico = 0,02 * 1000 / 4 = 5
//KDaplicado = 0    => KDteorico = 0 * 4 / 1000 = 0

//Para una GANANCIA=11,11 , SENSIBILIDAD = 3 y KP = 2,85 , los parametros serian:
//KIteorico = 1,2 => KIaplicado = 1,2 * 4 / 1000 = 0.0048
//KDteorico = 1,8 => KDaplicado = 1,8 * 1000 / 4 = 450
//KPyaw  = 5.0
//KIteorico = 1.0 => KIaplicado = 1,0 * 4 / 1000 = 0,004
//KDteorico = 0.1 => KDaplicado = 0,1 * 1000 / 4 = 25

//Para una GANANCIA=15 , SENSIBILIDAD = 3 y KP = 1,8 , los parametros serian:
//KIteorico = 6,5 => KIaplicado = 6,5 * 4 / 1000 = 0.026
//KDteorico = 0,12 => KDaplicado = 0,12 * 1000 / 4 = 30,0
//KPyaw  = 5.0
//KIteorico = 0.0 => KIaplicado = 0,0 * 4 / 1000 = 0,0
//KDteorico = 0.0 => KDaplicado = 0,0 * 1000 / 4 = 0,0

float calculatePitchPID (int referencia, float angulo, float velocidad) {
    //referencia = prop.joystickY | SetpointPitch
    //angulo = angle_pitch_output  
    //velocidad = angular_motion_pitch

    float error_ajustado = angulo * PITCH_GAIN; //calcular la correcion previa de los angulos
    float setpoint_ajustado = referencia - error_ajustado; //calcular el "setpoint" referencia inicial
    //Dividimos por la SENSIBILIDAD para obtener magnitudes "dps" o degree_per_second (a menor SENSIBILIDAD mayor agilidad).
    setpoint_ajustado /= SENSIBILITY;

    //componentes del controlador PID
    error_pitch = velocidad - setpoint_ajustado;
    error_proporcional = KPpitch * error_pitch;
    error_integral_pitch += ki_pitch * error_pitch; //ojo:ki_pitch o KIpitch
    error_integral_pitch = constrain (error_integral_pitch, -maxErrorPITCH, +maxErrorPITCH);
    error_derivado = kd_pitch * (error_pitch - last_error_pitch); //ojo:kd_pitch o KDpitch
    last_error_pitch = error_pitch;
    
    PIDpitch = error_proporcional + error_integral_pitch + error_derivado; 
    PIDpitch = constrain (PIDpitch, -maxErrorPITCH, +maxErrorPITCH);
    return PIDpitch;
}

float calculateRollPID (int referencia, float angulo, float velocidad) {
    //referencia = prop.joystickY | SetpointPitch
    //angulo = angle_roll_output  
    //velocidad = angular_motion_roll

    float error_ajustado = angulo * ROLL_GAIN; //calcular la correcion previa de los angulos
    float setpoint_ajustado = referencia - error_ajustado; //calcular el "setpoint" referencia inicial
    //Dividimos por la SENSIBILIDAD para obtener magnitudes "dps" o degree_per_second (a menor SENSIBILIDAD mayor agilidad).
    setpoint_ajustado /= SENSIBILITY;

    //componentes del controlador PID
    //¡¡¡OJO!!! error_roll = velocidad - setpoint_ajustado ¿porque?
    error_roll = setpoint_ajustado - velocidad;
    error_proporcional = KProll * error_roll;
    error_integral_roll += ki_roll * error_roll; //ojo:ki_roll o KIroll
    error_integral_roll = constrain (error_integral_roll, -maxErrorROLL, +maxErrorROLL);
    error_derivado = kd_roll * (error_roll - last_error_roll); //kd_roll o KDroll
    last_error_roll = error_roll;
    
    PIDroll = error_proporcional + error_integral_roll + error_derivado; 
    PIDroll = constrain (PIDroll, -maxErrorROLL, +maxErrorROLL);
    return PIDroll;
}

float calculateYawPID (int referencia, float angulo, float velocidad) {  
    //referencia = prop.joystickX | SetpointYaw
    //angulo = angle_yaw_output  
    //velocidad = angular_motion_yaw

    float error_ajustado = angulo * YAW_GAIN; //calcular la correcion previa de los angulos
    float setpoint_ajustado = referencia - error_ajustado; //calcular el "setpoint" referencia inicial
    //Dividimos por la SENSIBILIDAD para obtener magnitudes "dps" o degree_per_second (a menor SENSIBILIDAD mayor agilidad).
    //In the case of deviding by 3 the max yaw rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
    setpoint_ajustado /= SENSIBILITY;
  
    //componentes del controlador PID
    error_yaw = velocidad - setpoint_ajustado;
    error_proporcional = KPyaw * error_yaw;
    error_integral_yaw += ki_yaw * error_yaw;  //ojo:ki_yaw o KIyaw
    error_integral_yaw = constrain (error_integral_yaw, -maxErrorYAW, +maxErrorYAW);
    
    //integración trapezoidal para mejora calculo del término integral
    //----------------------------------------------------------------
    //error_integral_yaw = ki_yaw * (error_yaw + last_error_yaw) / 2;  //ojo:ki_yaw o KIyaw
    //error_integral_yaw = constrain (error_integral_yaw, -maxErrorYAW, +maxErrorYAW);
    
    error_integral_yaw = constrain (error_integral_yaw, -maxErrorYAW, +maxErrorYAW);
    error_derivado = kd_yaw * (error_yaw - last_error_yaw); //ojo:kd_yaw o KDyaw
    last_error_yaw = error_yaw;
    
    PIDyaw = error_proporcional + error_integral_yaw + error_derivado; 
    PIDyaw = constrain (PIDyaw, -maxErrorYAW, +maxErrorYAW);

    return PIDyaw;  
}



//****************************** METODOS TRADICIONALES ******************************
//NOTA: 
//(1) Una importante cuestion a dilucidar es si aplicar un coeficiente corrector debido a las asimetrias del modelo, 
//    a saber, incrementar en un porcentaje la salida de PWMs para la PROA debido a la menor longitud de su brazo respecto al CG, 
//    y decrementarle ese mismo porcentaje de PWMs al motor de POPA.
//(2) Idem de idem, ajustando las PWMs de cada motor con su "offset" producto de los diferentes "umbrales" de los motores.

float calculate2PitchPID (int referencia, float angulo) {
  float KPaplicado;

  error_pitch = (referencia/GAIN) - angulo;
  
  //aplicar la "agresividad"
  if ( abs(error_pitch) > aggPitchAngle)
        KPaplicado = aggKPpitch;
  else
        KPaplicado = KPpitch;
  
  error_proporcional = KPaplicado * error_pitch; //antes KPpitch

  error_integral_pitch += error_pitch;
  error_integral = error_integral_pitch * ki_pitch;
  //error_integral = constrain (error_integral, -maxErrorINTEGRAL, +maxErrorINTEGRAL);
  error_integral = constrain (error_integral, -maxErrorPITCH, +maxErrorPITCH);

  error_derivado = (error_pitch - last_error_pitch) * kd_pitch;
  last_error_pitch = error_pitch;

  #if TRAZAS_PITCH
        Serial.print(F("error_pitch:"));Serial.print(error_pitch);Serial.print("\t\t");
        Serial.print(F("error_proporcional:"));Serial.print(error_proporcional);Serial.print("\n");
        
        Serial.print(F("error_integral_pitch:"));Serial.print(error_integral_pitch);Serial.print("\t");
        Serial.print(F("error_integral:"));Serial.print(error_integral);Serial.print("\n");

        Serial.print(F("last_error_pitch:"));Serial.print(last_error_pitch);Serial.print("\t");
        Serial.print(F("error_derivado:"));Serial.print(error_derivado);Serial.print("\n\n");
  #endif

  PIDpitch = (error_proporcional + error_integral + error_derivado);
  PIDpitch = constrain (PIDpitch, -maxErrorPITCH, +maxErrorPITCH);

  #if PLOTTER_PITCH
        //Serial.print(error_proporcional);Serial.print(",");
        //Serial.print(error_integral);Serial.print(",");
        //Serial.print(error_derivado);Serial.print(",");
        //Serial.print(PIDpitch);Serial.print(\n");
        Serial.print(aggPitchAngle);Serial.print(",");
        Serial.print(abs(error_pitch));Serial.print(",");
        Serial.print(KPaplicado);Serial.print("\n");
  #endif

  return PIDpitch;
}

float calculate2RollPID (int referencia, float angulo) {
  float KPaplicado;

  error_roll = (referencia/GAIN) - angulo;
  
  //aplicar la "agresividad"
  if ( abs(error_roll) > aggRollAngle)
        KPaplicado = aggKProll;
  else
        KPaplicado = KProll;

  error_proporcional = KPaplicado * error_roll; //antes KPpitch

  error_integral_roll += error_roll;
  error_integral = error_integral_roll * ki_roll; 
  //error_integral = constrain (error_integral, -maxErrorINTEGRAL, +maxErrorINTEGRAL);
  error_integral = constrain (error_integral, -maxErrorROLL, +maxErrorROLL);
  
  error_derivado = (error_roll - last_error_roll) * kd_roll;
  last_error_roll = error_roll;

  #if TRAZAS_ROLL
        Serial.print(F("error_roll:"));Serial.print(error_roll);Serial.print("\t\t");
        Serial.print(F("error_proporcional:"));Serial.print(error_proporcional);Serial.print("\n");
        
        Serial.print(F("error_integral_roll:"));Serial.print(error_integral_roll);Serial.print("\t");
        Serial.print(F("error_integral:"));Serial.print(error_integral);Serial.print("\n");

        Serial.print(F("last_error_roll:"));Serial.print(last_error_roll);Serial.print("\t");
        Serial.print(F("error_derivado:"));Serial.print(error_derivado);Serial.print("\n\n");
  #endif

  PIDroll = (error_proporcional + error_integral + error_derivado);
  PIDroll = constrain (PIDroll, -maxErrorROLL, +maxErrorROLL);

  #if PLOTTER_ROLL
        //Serial.print(error_proporcional);Serial.print(",");
        //Serial.print(error_integral);Serial.print(",");
        //Serial.print(error_derivado);Serial.print(",");
        //Serial.print(PIDroll);Serial.print(\n");
        Serial.print(aggRollAngle);Serial.print(",");
        Serial.print(abs(error_roll));Serial.print(",");
        Serial.print(KPaplicado);Serial.print("\n");
  #endif

  return PIDroll;
}

float calculate2YAWMotionPID (int referencia, float velocidad) {
  //-----------------------------------------------------------------------------------------------------------------------------------------------------
  //El calculo del PIDyaw, segun la mayoria de autores, siempre estara basado en valores del "gyro" y por tanto en velocidades angulares y no en angulos.
  //-----------------------------------------------------------------------------------------------------------------------------------------------------
  
  //error_yaw = referencia - velocidad; //antiguo
  error_yaw = velocidad - 1.0*referencia;
  //NO aplicar la "agresividad"; siempre se es agresivo con el YAW (fenomeno SPINNING-TO-THE-MOON).
  error_proporcional = KPyaw * error_yaw;

  //error_integral_yaw += error_yaw; //antiguo
  //error_integral = error_integral_yaw * ki_yaw; //antiguo
  error_integral += ki_yaw * error_yaw;
  error_integral = constrain (error_integral, -maxErrorYAW, +maxErrorYAW);

  error_derivado = (error_yaw - last_error_yaw) * kd_yaw; //alternativo
  last_error_yaw = error_yaw;

  #if TRAZAS_YAW
        Serial.print(F("error_yaw:"));Serial.print(error_yaw);Serial.print("\t\t");
        Serial.print(F("error_proporcional:"));Serial.print(error_proporcional);Serial.print("\n");        
        Serial.print(F("error_integral:"));Serial.print(error_integral);Serial.print("\n");
        Serial.print(F("error_derivado:"));Serial.print(error_derivado);Serial.print("\n\n");
        Serial.print(F("last_error_yaw:"));Serial.print(last_error_yaw);Serial.print("\t");
  #endif

  PIDyaw = (error_proporcional + error_integral + error_derivado);
  PIDyaw = constrain (PIDyaw, -maxErrorYAW, +maxErrorYAW);

  #if PLOTTER_YAW
        Serial.print(error_proporcional);Serial.print(",");
        Serial.print(error_integral);Serial.print(",");
        Serial.print(error_derivado);Serial.print(",");
        Serial.print(PIDyaw);Serial.print("\n");
  #endif

  return PIDyaw;
}

float calculate2YAWPositionPID (int referencia, float angulo) {
  //error_yaw = -1.0 * referencia - angulo; //metodo antiguo y clasico
  error_yaw = angulo - 1.0*referencia;
  error_proporcional = KPyaw * error_yaw;

  error_integral_yaw += error_yaw;
  error_integral = error_integral_yaw * ki_yaw;
  error_integral = constrain (error_integral, -maxErrorYAW, +maxErrorYAW);

  error_derivado = (error_yaw - last_error_yaw) * kd_yaw;
  last_error_yaw = error_yaw;

  #if TRAZAS_YAW
        Serial.print(F("error_yaw:"));Serial.print(error_yaw);Serial.print("\t\t");
        Serial.print(F("error_proporcional:"));Serial.print(error_proporcional);Serial.print("\n");
        
        Serial.print(F("error_integral_yaw:"));Serial.print(error_integral_yaw);Serial.print("\t");
        Serial.print(F("error_integral:"));Serial.print(error_integral);Serial.print("\n");

        Serial.print(F("last_error_yaw:"));Serial.print(last_error_yaw);Serial.print("\t");
        Serial.print(F("error_derivado:"));Serial.print(error_derivado);Serial.print("\n\n");
  #endif

  PIDyaw = (error_proporcional + error_integral + error_derivado);
  PIDyaw = constrain (PIDyaw, -maxErrorYAW, +maxErrorYAW);

  #if PLOTTER_YAW
        Serial.print(error_proporcional);Serial.print(",");
        Serial.print(error_integral);Serial.print(",");
        Serial.print(error_derivado);Serial.print(",");
        Serial.print(PIDyaw);Serial.print("\n");
  #endif

  return PIDyaw;
}



//****************************** METODOS MODERNOS ******************************
//(1) DERIVATIVE KICK: 
//--------------------
//   Este fenomeno se produce por variaciones rápidas en la señal de referencia o "Setpoint", que se magnifican por la acción derivativa se transforman 
//en componentes transitorios de gran amplitud en la señal de control. Cualquier cambio en la consigna, causa un cambio instantáneo en el error y la 
//derivada de este cambio puede ser infinito (en la práctica, dt no es cero, pero igualmente el valor termina siendo muy grande). 
//   Esto produce un sobrepico muy alto en la salida, que podemos corregir de una manera muy sencilla. Resulta que la derivada del error es igual a la 
//derivada negativa de la entrada, salvo cuando el "Setpoint" está cambiando y esto acaba siendo una solución perfecta. En lugar de añadir 
//(Kd * error_derivado), restamos (Kd * valor_de_entrada_derivado). 
//   Esto se conoce como el uso de DERIVADA DE LA MEDICION. Las modificaciones son bastante sencillas, reemplazando la derivada POSITIVA del error con la 
//derivada NEGATIVA de la entrada. En vez de recordar el último valor del error, ahora recordamos el último valor que tomó la entrada.

//(2) KP/KI/KD VARIABLES: 
//-----------------------
//   La posibilidad de cambiar la sintonización del PID, mientras el sistema está corriendo, es la característica más respetable de algunos sistemas de 
//control (FUZZY-LOGIC). De inmediato podemos ver que el culpable de los ¿baches? en la señal de salida es el término integral ya que es el único 
//término que cámbia drásticamente cuando la señal de sintonización se modifica. De repente, la suma de todos los errores se multiplica con el valor de 
//Ki, y esto no es lo que necesitamos, ya que solo queremos que afecte a los valores que estén despues del cambio.
//   La solución a este error consiste en tener el término KI fuera de la integral. En la práctica está acción resulta en una gran diferencia en la 
//función de salida del PID. Tomamos el error y lo multiplicamos por el valor de KI en ese momento, luego almacenamos la suma de los diferentes errores
//multiplicados por la constante KI. Esto da una función de salida, suave y sin sobresaltos, con la ventaja de no tener que utilizar matemática 
//adicional para ello.

//(3) RESET WIND-UP: 
//------------------
//   El efecto windup aparece al arrancar el sistema o en cualquier otra situación donde aparece un error muy grande durante un tiempo prolongado. Esto 
//hará que el término integral aumente para reducir el error. Pero si nuestro actuador es limitado se saturará, pero el termino integral seguirá 
//creciendo. Cuando el error se reduce, la parte integral también comenzará a reducirse, pero desde un valor muy alto, llevando mucho tiempo hasta que 
//logre la estabilidad, generando fluctuaciones exageradamente grandes. El problema se manifiesta en forma de retrasos extraños. 
//   Hay varias formas para mitigar el efecto RESET WIND-UP; la elegida la siguiente: "decirle al PID cuáles son los límites de salida" o crear una 
//función SetOutputIntegralLimits. Una vez que ya se alcanza el límite, el PID detiene el funcionamiento del término integral.

float calculate3PitchPID (int referencia, float angulo) {  
  error_pitch = (float)(referencia/GAIN) - angulo;
  error_proporcional = KPpitch * error_pitch;

  //error_integral_pitch += error_pitch;
  //error_integral = error_integral_pitch * ki_pitch;
  error_integral_pitch += (ki_pitch * error_pitch);
  error_integral = error_integral_pitch;
  error_integral = constrain (error_integral, -maxErrorINTEGRAL, +maxErrorINTEGRAL);
  
  //error_derivado = (error_pitch - last_error_pitch) * kd_pitch;
  //last_error_pitch = error_pitch;
  error_derivado = -1.0 * (angulo - last_angulo_pitch) * kd_pitch;

  #if TRAZAS_PITCH
        Serial.print(F("error_pitch:"));Serial.print(error_pitch);Serial.print("\t\t");
        Serial.print(F("error_proporcional:"));Serial.print(error_proporcional);Serial.print("\n");
        
        Serial.print(F("error_integral_pitch:"));Serial.print(error_integral_pitch);Serial.print("\t");
        Serial.print(F("error_integral:"));Serial.print(error_integral);Serial.print("\n");

        //Serial.print(F("last_error_pitch:"));Serial.print(last_error_pitch);Serial.print("\t");
        Serial.print(F("angulos:"));Serial.print(angulo);Serial.print(" , ");Serial.print(last_angulo_pitch);Serial.print(" , ");Serial.print(angulo-last_angulo_pitch);Serial.print("\t");
        Serial.print(F("error_derivado:"));Serial.print(error_derivado);Serial.print("\n\n");
  #endif

  #if PLOTTER_PITCH
        Serial.print(error_proporcional);Serial.print(",");
        Serial.print(error_integral);Serial.print(",");
        Serial.print(error_derivado);Serial.print("\n");
  #endif

  last_angulo_pitch = angulo;

  PIDpitch = (error_proporcional + error_integral + error_derivado);
  PIDpitch = constrain (PIDpitch, -maxErrorPITCH, +maxErrorPITCH);
  
  return PIDpitch;
}

float calculate3RollPID (int referencia, float angulo) {
  error_roll = (float)(referencia/GAIN) - angulo;
  error_proporcional = KProll * error_roll;

  //error_integral_roll += error_roll;
  //error_integral = error_integral_roll * ki_roll;
  error_integral_roll += (ki_roll * error_roll);
  error_integral = error_integral_roll;
  error_integral = constrain (error_integral, -maxErrorINTEGRAL, +maxErrorINTEGRAL);

  //error_derivado = (error_roll - last_error_roll) * kd_roll;
  //last_error_roll = error_roll;
  error_derivado = -1.0 * (angulo - last_angulo_roll) * kd_roll;

  #if TRAZAS_ROLL
        Serial.print(F("error_roll:"));Serial.print(error_roll);Serial.print("\t\t");
        Serial.print(F("error_proporcional:"));Serial.print(error_proporcional);Serial.print("\n");
        
        Serial.print(F("error_integral_roll:"));Serial.print(error_integral_roll);Serial.print("\t");
        Serial.print(F("error_integral:"));Serial.print(error_integral);Serial.print("\n");

        //Serial.print(F("last_error_roll:"));Serial.print(last_error_roll);Serial.print("\t");
        Serial.print(F("angulos:"));Serial.print(angulo);Serial.print(" , ");Serial.print(last_angulo_roll);Serial.print(" , ");Serial.print(angulo-last_angulo_roll);Serial.print("\t");
        Serial.print(F("error_derivado:"));Serial.print(error_derivado);Serial.print("\n\n");
  #endif

  #if PLOTTER_ROLL
        Serial.print(error_proporcional);Serial.print(",");
        Serial.print(error_integral);Serial.print(",");
        Serial.print(error_derivado);Serial.print("\n");
  #endif

  last_angulo_roll = angulo;

  PIDroll = (error_proporcional + error_integral + error_derivado);
  PIDroll = constrain (PIDroll, -maxErrorROLL, +maxErrorROLL);
  
  return PIDroll;  
}

float calculate3YawPID (int referencia, float angulo) {
  error_yaw = (float)(referencia/GAIN) - angulo;
  error_proporcional = KPyaw * error_yaw;

  //error_integral_yaw += error_yaw;
  //error_integral = error_integral_yaw * ki_yaw;
  error_integral_yaw += (ki_yaw * error_yaw);
  error_integral = error_integral_yaw;
  error_integral = constrain (error_integral, -maxErrorINTEGRAL, +maxErrorINTEGRAL);

  //error_derivado = (error_yaw - last_error_yaw) * kd_yaw;
  //last_error_yaw = error_yaw;
  error_derivado = -1.0 * (angulo - last_angulo_yaw) * kd_yaw;

  #if TRAZAS_YAW
        Serial.print(F("error_yaw:"));Serial.print(error_yaw);Serial.print("\t\t");
        Serial.print(F("error_proporcional:"));Serial.print(error_proporcional);Serial.print("\n");
        
        Serial.print(F("error_integral_yaw:"));Serial.print(error_integral_yaw);Serial.print("\t");
        Serial.print(F("error_integral:"));Serial.print(error_integral);Serial.print("\n");

        //Serial.print(F("last_error_yaw:"));Serial.print(last_error_yaw);Serial.print("\t");
        Serial.print(F("angulos:"));Serial.print(angulo);Serial.print(" , ");Serial.print(last_angulo_yaw);Serial.print(" , ");Serial.print(angulo-last_angulo_yaw);Serial.print("\t");
        Serial.print(F("error_derivado:"));Serial.print(error_derivado);Serial.print("\n\n");
  #endif

  #if PLOTTER_YAW
        Serial.print(error_proporcional);Serial.print(",");
        Serial.print(error_integral);Serial.print(",");
        Serial.print(error_derivado);Serial.print("\n");
  #endif

  last_angulo_yaw = angulo;

  PIDyaw = (error_proporcional + error_integral + error_derivado);
  PIDyaw = constrain (PIDyaw, -maxErrorYAW, +maxErrorYAW);
  
  return PIDyaw;    
}

float calculate3AltitudePID (float referencia, float altitude) {
  error_altitude = (referencia - altitude);
  ( error_altitude > 0 ) ? KPaltitude = KPASCaltitude : KPaltitude = KPDESaltitude; 
  error_proporcional = KPaltitude * error_altitude;

  //error_integral_altitude += error_altitude;
  //error_integral = error_integral_altitude * ki_altitude;
  //error_integral = constrain (error_integral, -maxErrorINTEGRAL, +maxErrorINTEGRAL);
  error_integral_altitude += (ki_altitude * error_altitude);
  error_integral = error_integral_altitude;
  error_integral = constrain (error_integral, -maxErrorINTEGRAL, +maxErrorINTEGRAL);

  //error_derivado = (error_altitude - last_error_altitude) * kd_altitude;
  //last_error_altitude = error_altitude;
  error_derivado = -1.0 * (altitude - last_altitude) * kd_altitude;
  last_altitude = altitude;

  #if TRAZAS_ALTITUDE
        Serial.print(F("error_altitude:"));Serial.print(error_altitude);Serial.print("\t\t");
        Serial.print(F("error_proporcional:"));Serial.print(error_proporcional);Serial.print("\n");
        
        Serial.print(F("error_integral_altitude:"));Serial.print(error_integral_altitude);Serial.print("\t");
        Serial.print(F("error_integral:"));Serial.print(error_integral);Serial.print("\n");

        Serial.print(F("last_error_altitude:"));Serial.print(last_error_roll);Serial.print("\t");
        Serial.print(F("error_derivado:"));Serial.print(error_derivado);Serial.print("\n\n");
  #endif

  PIDaltitude = (error_proporcional + error_integral + error_derivado);
  PIDaltitude = constrain (PIDaltitude, -maxErrorALTITUDE, +maxErrorALTITUDE);

  #if PLOTTER_ALTITUDE
        Serial.print(error_proporcional);Serial.print(",");
        Serial.print(error_integral);Serial.print(",");
        Serial.print(error_derivado);Serial.print(",");
        Serial.print(PIDaltitude);Serial.print("\n");
  #endif

  return PIDaltitude;
}



//****************************** FUNCIONES ESPECIALIZADAS ******************************
float calculateOnlyGyroPID (int referencia) {
    //El entero "referencia" siempre es el "prop.joystickX", ya que el parametro "SetpointYaw" ha sido eliminado  
    
    //Estas se modularan dividiendolas por la SENSIBILIDAD_MPU6050 = 3.0 de nuestro modelo.
    //FS_SEL_1=500º/s / 3.0 = 166,66dps, y a mayor sensibilidad dividiremos por un numero menor. 
    //return (float) ( (1.0 * referencia) /  SENSIBILIDAD_MPU6050 ); 

    //Para posiciones los valores se dividiran por la GANANCIA = 11.111
    //1 grado = 11,111 pwm (ya que (2500-500)pwm son 180 grados) => 250 pwm = 22,5 grados
    //return (float) ( (1.0 * referencia) / GANANCIA );

    //El valor del JOYSTICK se dividira por el DRCG (Damped Rate of Change Gain)
    //El DRCG sustituye de alguna manera al parametro SENSIBILITY
    //return (float) (1.0 * referencia) / (1.0 * DRCG);

    return (float) (1.0 * (referencia * DRCG));
}
