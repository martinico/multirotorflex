
#ifndef GLOBALES_H
  #define GLOBALES_H

  //---------- COMUNICACIONES ----------
  const char* ssid      = "MOVISTAR_9D44";
  const char* password  = "22CDE47821EACD24DD7B";
  const char* ssid2     = "MultiRotorFlex";
  const char* password2 = "";
  //router => 192.168.1.1/29a52422
  IPAddress ip (10, 0, 0, 1);
  IPAddress gateway(10, 0, 0, 1);
  IPAddress subnet(255, 255, 255, 0);
  IPAddress broadcast(255, 255, 255, 255);
  bool WS1conectado = false;
  bool WS2conectado = false;
  //------------------------------------
  
  bool newGPSNEO = false;
  String jsonGPSNEO;
  
  bool newWIFI = false;
  String jsonWIFI;
  
  bool newLIPO = false;
  String jsonLIPO;

  bool newADJUST = false;
  String jsonADJUST;

  bool newSYNC = false;
  String jsonSYNC;
  
  //Definicion de la maquina de estados
  #define PARADO     0
  #define TEST       1
  #define ENAIRE     2
  int status = PARADO;
  bool testing = true;
  bool volando = false;
  bool yawing = false;  
  bool holding = false;
  
  //Serial Plotter/Console Arduino 
  int pltPitch, pltRoll, pltRollEspecial, pltYaw, pltAltitude, plt_ESC_PROA, plt_ESC_POPA, plt_ESC_BABOR, plt_ESC_ESTRIBOR, plt_SERVO;
  bool newPlt;

  //Flag de reset para el gyroscopio, acelerometro, barometro, magnetometro, ... y controlador PID, cuando empezamos a volar.
  bool started = false;
  bool adjusted = true;
  
  //Valores calibrados manualmente tras un reset y toma aleatoria de valores con efecto Coriolis
  //float calibManualPitch = -0.47;
  //float calibManualRoll = 6.66;
  //float calibManualYaw = 0.0;

  bool sensoresCalibrados_250HZ = false;
  bool sensoresCalibrados_100HZ = false;

  //Calibracion de CPUs, procesos y tareas.
  int tiemposBuclePrincipal_250HZ = 0; //"flag" de activacion 
  unsigned long t1_250HZ, t2_250HZ;
  unsigned long MEDIA_250HZ = 0;
  //unsigned long MAX_250HZ = 0;
  //unsigned long MIN_250HZ = 99999; //watermark
  //Calibracion de CPUs, procesos y tareas.
  int tiemposBuclePrincipal_100HZ = 0; //"flag" de activacion 
  unsigned long t1_100HZ, t2_100HZ;
  unsigned long MEDIA_100HZ = 0;    
  //unsigned long MAX_100HZ = 0;
  //unsigned long MIN_250HZ = 99999; //watermark

  //Parametros que usaremos provisionalmente para pruebas como simples escalones para el aumento/disminucion de las PWMs, 
  //y para posteriormente y definitivamente, usarlos para el control PID en altitud.
  float FLL = 0.5;  //Flight Level Logic FLL
  float PARACHUTE = -0.5; //Efecto compensacion termico
  int nFLL = 0; //Numero de FLLs
  int HOLD = 0; //NIVELES a sostener usando un PID especifico

  float EMPUJE_ESPECIFICO = 2.275; // peso/(1450-1090) = 820/360 = 2,278 gramos/PWM

#endif
