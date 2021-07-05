
#include <Arduino.h>
#include "HMC5883L.h"
#include "Wire.h"

#define HMC_ADDRESS 0x0D   //I2C address of the HMC5883L

#define SDA1 4
#define SCL1 5

int16_t compass_x, compass_y, compass_z;
bool compass_calibration = false;
float actual_compass_heading = 0.0;
const float DECLINATION = 0.0;
float compass_x_horizontal, compass_y_horizontal;
int16_t compass_offset_x, compass_offset_y, compass_offset_z;
float compass_scale_y, compass_scale_z;
int16_t compass_calibration_values[6];

void read_HMC5883L() {
  Wire1.beginTransmission(HMC_ADDRESS);
  Wire1.write(0x03);  //we want to start reading at the hexadecimal location 0x03.
  Wire1.endTransmission();

  Wire1.requestFrom(HMC_ADDRESS, 6);              //Request 6 bytes from the compass.
  compass_y = Wire1.read() << 8 | Wire1.read();   //Add the low and high byte to the compass_y variable.
  compass_y *= -1;                                //Invert the direction of the axis.
  compass_z = Wire1.read() << 8 | Wire1.read();   //Add the low and high byte to the compass_z variable.;
  compass_x = Wire1.read() << 8 | Wire1.read();   //Add the low and high byte to the compass_x variable.;
  compass_x *= -1;                                //Invert the direction of the axis.

  //Before the compass can give accurate measurements it needs to be calibrated. 
  //At startup the compass_offset and compass_scale variables are calculated. 
  //The following part will adjust the raw compas values so they can be used for the calculation of the heading.
  if (!compass_calibration) {
    compass_y += compass_offset_y;    //Add the y-offset to the raw value.
    compass_y *= compass_scale_y;     //Scale the y-value so it matches the other axis.
    compass_z += compass_offset_z;    //Add the z-offset to the raw value.
    compass_z *= compass_scale_z;     //Scale the z-value so it matches the other axis.
    compass_x += compass_offset_x;    //Add the x-offset to the raw value.
  }

  //The compass values change when the roll and pitch angle of the quadcopter changes. 
  //That's the reason that the x and y values need to calculated for a virtual horizontal position.
  //The 0.0174533 value is phi/180 as the functions are in radians in stead of degrees.
  float angle_roll, angle_pitch = 0.0; //¡OJO CON EL TRUCO!
  compass_x_horizontal = (float)compass_x * cos(angle_pitch * -0.0174533) + (float)compass_y * sin(angle_roll * 0.0174533) * sin(angle_pitch * -0.0174533) - (float)compass_z * cos(angle_roll * 0.0174533) * sin(angle_pitch * -0.0174533);
  compass_y_horizontal = (float)compass_y * cos(angle_roll * 0.0174533) + (float)compass_z * sin(angle_roll * 0.0174533);

  //Now that the horizontal values are known the heading can be calculated. 
  //With the following lines of code the heading is calculated in degrees.
  //Please note that the atan2 uses radians in stead of degrees. That is why the 180/3.14 is used.
  if (compass_y_horizontal < 0)actual_compass_heading = 180 + (180 + ((atan2(compass_y_horizontal, compass_x_horizontal)) * (180 / 3.14)));
  else actual_compass_heading = (atan2(compass_y_horizontal, compass_x_horizontal)) * (180 / 3.14);

  actual_compass_heading += DECLINATION; //OBTENCION DEL NORTE GEOGRAFICO

  //criterio YMFC
  if (actual_compass_heading < 0.0)
        actual_compass_heading += 360.0;
  if (actual_compass_heading >= 360.0)
        actual_compass_heading -= 360.0;

  //criterio MSM
  if (actual_compass_heading > 180.0) {
        actual_compass_heading = actual_compass_heading - 360.0;
  } else if (actual_compass_heading <= -180.0)  {
        actual_compass_heading = actual_compass_heading + 360.0;
  }

}

void test_HMC5883L() {
    Wire1.beginTransmission(HMC_ADDRESS);
    Wire1.write(0x00);
    Wire1.write(0x10);
    Wire1.write(0x60);
    Wire1.write(0x00);
    Wire1.endTransmission();    
    delay(2000);

    Serial.print("Positive bias test: ");
    Wire1.beginTransmission(HMC_ADDRESS);
    Wire1.write(0x00);
    Wire1.write(0x11);
    Wire1.write(0x60);
    Wire1.write(0x01);
    Wire.endTransmission();
    delay(10);
    read_HMC5883L();
    Serial.print(" x: ");  Serial.print(compass_x);
    Serial.print(" y: ");  Serial.print(compass_y);
    Serial.print(" z: ");  Serial.println(compass_z);

    Serial.print("Negative bias test: ");
    Wire1.beginTransmission(HMC_ADDRESS);
    Wire1.write(0x00);
    Wire1.write(0x12);
    Wire1.write(0x60);
    Wire1.write(0x01);
    Wire1.endTransmission();
    delay(10);
    read_HMC5883L();
    Serial.print(" x: ");  Serial.print(compass_x);
    Serial.print(" y: ");  Serial.print(compass_y);
    Serial.print(" z: ");  Serial.println(compass_z);

    Wire1.beginTransmission(HMC_ADDRESS);
    Wire1.write(0x00);
    Wire1.write(0x10);
    Wire1.write(0x60);
    Wire1.write(0x00);
    Wire1.endTransmission();
    delay(2000);
}

void init_HMC5883L() {   
  
}

void init_HMC5883L_ok() {
  Wire1.begin(SDA1, SCL1);
  Wire1.setClock(400000); // 400kHz I2C clock (comment this line if having compilation difficulties)
  Wire1.beginTransmission(HMC_ADDRESS);
  uint8_t error = Wire1.endTransmission();
  while (error != 0) {
    Serial.print("**** ERROR HMC5883L : ");
    Serial.println(error);
    delay(1000);
  }
  
  #define LED_PIN 2 // Turn LED on during setup. 13->Arduino, 2->ESP32
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  
  //setup de los registros
  Wire1.beginTransmission(HMC_ADDRESS);
  Wire1.write(0x00);  //We want to write to the Configuration Register A (00 hex).
  Wire1.write(0x78);  //Set the Configuration Regiser A bits as 01111000 to set sample rate (average of 8 at 75Hz).
  Wire1.write(0x20);  //Set the Configuration Regiser B bits as 00100000 to set the gain at +/-1.3Ga.
  Wire1.write(0x00);  //Set the Mode Regiser bits as 00000000 to set Continues-Measurement Mode.
  Wire1.endTransmission();
    
  //for (int error = 0; error < 6; error ++)compass_calibration_values[error] = EEPROM.read(0x10 + error); //read the calibration values from the EEPROM

  //Calculate the calibration offset and scale values
  compass_scale_y = ((float)compass_calibration_values[1] - compass_calibration_values[0]) / (compass_calibration_values[3] - compass_calibration_values[2]);
  compass_scale_z = ((float)compass_calibration_values[1] - compass_calibration_values[0]) / (compass_calibration_values[5] - compass_calibration_values[4]);
  compass_offset_x = (compass_calibration_values[1] - compass_calibration_values[0]) / 2 - compass_calibration_values[1];
  compass_offset_y = (((float)compass_calibration_values[3] - compass_calibration_values[2]) / 2 - compass_calibration_values[3]) * compass_scale_y;
  compass_offset_z = (((float)compass_calibration_values[5] - compass_calibration_values[4]) / 2 - compass_calibration_values[5]) * compass_scale_z;

  read_HMC5883L();

  Serial.print("\n actual_compass_heading:"); Serial.println(actual_compass_heading);

  //angulo de yaw => actual_compass_heading

  digitalWrite(LED_PIN, LOW);
}



void HMC5883L_calculator() {
  
}
