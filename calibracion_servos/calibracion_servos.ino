
// ZOOMKAT 3-28-14 serial servo incremental test code
// Using serial monitor type a char ("s" to increase, "a" to decrease) and enter to change the servo position
// Use strings like 90x or 1500x for new servo position

#include <ESP32Servo.h> //¡¡¡¡ ojo #include <Servo.h> para Arduino-UNO !!!
String readString;
Servo myservo;
int pos=1500; //~neutral value for continuous rotation servo
//int pos=90;

void setup() {
  myservo.setPeriodHertz(250); //quitar para Arduino-UNO
  myservo.attach(33, 1400, 2600); //27=ESCproa 26=SERVOproa 32=ESCpopa  33=SERVOproa
  delay(100);
  Serial.begin(115200);
  Serial.println("serial servo incremental test code");
  Serial.println("----------------------------------");
  Serial.println("Type a character (s to increase or a to decrease) and ENTER to change servo position");
  Serial.println("Use strings like 90x or 1500x for new servo position");
  Serial.println();
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();  //gets one byte from serial buffer
    readString += c; //makes the string readString
    delay(2);  //slow looping to allow buffer to fill with next character
  }
  
  if (readString.length() >0) {
    if(readString.indexOf('x') >0) {
      pos = readString.toInt();
    }
    if (readString == "a"){
      (pos=pos-1); //use larger numbers for larger increments
      if(pos<0) (pos=0); //prevent negative number
    }
    if (readString == "s"){
      (pos=pos+1);
    }
    if(pos >= 400){ //determine servo write method
      Serial.println(pos);
      myservo.writeMicroseconds(pos);
    } else {   
      Serial.println(pos);
      myservo.write(pos);
    }
  }
  readString=""; //empty for next input
}

//RDS3115 = 400 / 2585 => con un valor superior, se queda colgado, con valor inferiores no responde, pero sigue funcionando.

/*
GENERAL SPECIFICATION
---------------------
Storage temperature range     -20°C ~ 60°C
Operating temperature range   -10°C ~ 50°C
Operating voltage             5V ~ 7.2V

ELECTRICAL SPECIFICATION
------------------------
Description                   5v            7.2v
Operating speed (at no load)  0.16sec/60°   0.14sec/ 60°
Running current (at no load)  80mA          100mA
Stall torque (at lock)        13.5kg.cm     15kg.cm
Stall current (at lock)       1.3A          1.5A
Idle current (at stopped)     4mA           5mA

MECHANICAL SPECIFICATION
------------------------
Overall dimension             40 x 20 x 40.5mm
Limit Angle                   360°±10°
Weight                        64±1g
Connector wire gauge          #28 PVC
Connector wire length         320±5mm
Horn gear spline              25T/ψ5.80
Reduction ratio/td            310:1

CONTROL SPECIFICATION
---------------------
Operating frequency           50-330Hz
Operating angle               90° (from 1000 to 2000 usec)
Neutral position              1500 usec
Dead band with                3 usec
Rotating direction            Counter clockwise (from 1000 to 2000 usec)
Pulse width range             From 500 to 2500 usec

PACKAGE INCLUDES
----------------
1 x 13.5kg.cm Robot Standard Servo RDS3115 Metal Gear Digital Servo -180 degree
*/
