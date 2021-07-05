
#ifndef propulsion_H
#define propulsion_H

  extern int HOVER;
  
  struct datosPROP {
    //************************** ESCs **************************
    const unsigned int UMBRAL_PROA = 1077;      
    const unsigned int OFFSET_PROA = 0;
    const unsigned int UMBRAL_POPA = 1083;      
    const unsigned int OFFSET_POPA = 0;
    const unsigned int UMBRAL_ESTRIBOR = 1088;  
    const unsigned int OFFSET_ESTRIBOR = 0;  
    const unsigned int UMBRAL_BABOR = 1082;     
    const unsigned int OFFSET_BABOR = 0;

    const unsigned int UMBRAL = 1090;
    const unsigned int OFFSET = 10;

    const unsigned int LIMITE_ESC = 1900;
    
    unsigned int throttle = 1000; //valor suinistrado de THROTTLE
    unsigned int joystickX = 0; //valor suministrado por JOYSTCIK
    unsigned int joystickY = 0; //valor suministrado por JOYSTCIK
    unsigned int pwmPROA; //valor real (tras PID) suministrado por los ESC al motor BLDC
    unsigned int pwmPOPA; //valor real (tras PID) suministrado por los ESC al motor BLDC
    unsigned int pwmBABOR; //valor real (tras PID) suministrado por los ESC al motor BLDC
    unsigned int pwmESTRIBOR; //valor real (tras PID) suministrado por los ESC al motor BLDC

    //************************** SERVOs **************************
    //1 grado = 11,111 pwm (ya que (2500-500)pwm son 180 grados) => 250 pwm = 22,5 grados
    //1 milisec = 0,4 grados (ya que 60 grados son 150 milis) => 4 milisec => 1,6 grados

    const unsigned int LIMITE_SERVO = 250;
    
    const unsigned int ALIGN_SERVO = 1580; //AJUSTE MANUAL
    const int OFFSET_SERVO = 80; //+=derechas | -=izquierdas

    int anguloSERVO; //valor real (tras PID) suministrado al RDS3115
  };
  extern datosPROP prop;

  //Funciones de bajo nivel
  void init_propulsion ();
  void testPROA ();
  void testPOPA ();
  void testBABOR ();
  void testESTRIBOR ();
  void testSERVO ();
  void pwm_init(int P, int C, int T, uint32_t duty);
  void pwm_set(uint32_t duty, int C);
  void setTestProp (int P1, int P2, int P3, int P4, int P5);
  void setProp ();
  void setPROA ();
  void setPOPA ();
  void setESTRIBOR ();
  void setBABOR ();
  void setServo ();

  //Funciones propias del ESP32
  void init_propulsion_ledc ();
  void set_prop_ledc ();

#endif

/*
RDS315:
-------
0,14sec/60º/7v versus 0,16sec/60º/5v
(2600pwm - 400pwm) son 180º => 1º son 12,222pwm
(2500pwm - 500pwm) son 180º => 1º son 11,111pwm

60º en 0,15sec => 1º en 2,5msec => 1msec son 0,4º y 4,888pwm (practiamente 5pwm)

Electrical Specification:     5V            7.2V
Operating speed (at no load)  0.16sec/ 60°  0.14sec/ 60°
Running current (at no load)  80mA          100mA
Stall torque (at lock)        13.5kg.cm     15kg.cm
Stall current (at lock)       1.3A          1.5A
Idle current (at stopped)     4mA           5mA
Mechanical Specification:
Overall dimension             40 x 20 x 40.5mm
Limit Angle                   360°±10°
Weight                        64±1g
Connector wire gauge          #28 PVC
Connector wire length         320±5mm
Horn gear spline              25T/ψ5.80
Reduction ratio               310:1
Control Specification:
Operating frequency           50-330Hz
Operating angle               135° (from 1000 to 2000 usec)
Neutral position              1500 usec
Dead band with                3 usec
Rotating direction            Counter clockwise (from 1000 to 2000 usec)
Pulse width range             From 500 to 2500 usec
*/

/*
Regimen motor para el vuelo:
----------------------------
Rango de temperaturas = 29,0 - 30,5 ºC
Rango de presiones = 1011,1 - 1012,6 mb
1445 PWM => HOVER con efecto suelo
1490 PWM => HOVER para el vuelo libre
1505-1515 PWM => HOVER con resistencia al cambio de altitud
SENSOR FUSION PITCH & ROLL = 0,9997
PITCH GAIN = 10.0
ROLL GAIN = 10.0
YAW GAIN = 10.0
FLIGHT LEVEL LOGIC = 10
PARACHUTE = -25
*/
