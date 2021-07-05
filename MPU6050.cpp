
#include <Arduino.h>
#include "MPU6050.h"
#include "Wire.h"

#define MPU_ADDRESS 0x68   //I2C address of the MPU-6050
#define SSF_GYRO_0  131    //FS_SEL=0 (250º/s)
#define SSF_GYRO_1  65.5   //FS_SEL=1 (500º/s)
#define SSF_GYRO_2  32.8   //FS_SEL=1 (1000º/s)
#define SSF_GYRO_3  16.4   //FS_SEL=1 (2000º/s)
#define FREQ        250    //Sampling-Frequency ¿extern?

float DRIFT             = 0.9970;
float DRIFT_PITCH       = 0.9800;
float DRIFT_ROLL        = 0.9800;
float DRIFT_PITCH_ROLL  = 0.9950;
float DRIFT_YAW         = 0.9996;

const float HPF = 0.9; //Complementary Filter to dampen the pitch & roll angles
const float LPF = 0.7; //LOW-PASS Filter (10Hz cutoff frequency) to the motion

// CALIBRACION
int contador_calibracion;
float pitch_calibrado = -0.44;
float roll_calibrado = 7.15;
float pitch_calibrado_temp = 0.0;
float roll_calibrado_temp = 0.0;

int FiltroFusionSensores = 0;

//Gyro angle calculations using integration => 1 / (FREQ / SSF_GYRO_1) where SSF_GYRO_1 = 65.5 or FS_SEL=1 (500º/s)
// FOR 200Hz (5 milisec) => 1 / 250.0 / 65.5 = 0.0000763
// FOR 250Hz (4 milisec) => 1 / 250.0 / 65.5 = 0.0000611
// FOR 333Hz (3 milisec) => 1 / 333.3 / 65.5 = 0.0000458
// FOR 500Hz (2 milisec) => 1 / 500.0 / 65.5 = 0.0000305
double integrator = 0.0000611;  // FOR 250Hz => 1 / 250.0 / 65.5 = 0.0000611

//********** RESULTADOS DE LOS TESTS **********
//angle_pitch/roll = angle_pitch/roll * DRIFT(casi 1.0)   +   angle_pitch/roll_acc * (1.0 - DRIFT) (casi 0.0)
//                   GIROSCOPIO                               ACELEROMETRO
//                   velocidad                                inclinaciones
//                   rapido con deriva                        lento, sin deriva y con ruido
//                   bueno para calculos PID                  bueno para visualizaciones H.A. y resets

// MPU variables
int16_t gyro_x, gyro_y, gyro_z;
int16_t acc_x, acc_y, acc_z;
int long gyro_x_cal, gyro_y_cal, gyro_z_cal;
int long acc_x_cal, acc_y_cal, acc_z_cal;

int long acc_total_vector;

float angle_pitch, angle_roll, angle_yaw;

float angle_roll_acc, angle_pitch_acc;
float angle_pitch_acc_calibrado, angle_roll_acc_calibrado;

boolean set_gyro_angles;

float angle_pitch_output, angle_roll_output, angle_yaw_output = 0.0;
float angular_motion_pitch, angular_motion_roll, angular_motion_yaw = 0.0;

#define TRAZAS_ANGULOS 0
#define PLOTTER_ANGULOS 0
#define TRAZAS_ANGULAR 0
#define PLOTTER_ANGULAR 0



void init_MPU6050 () {
  //I2C deault
  #define SDA 21
  #define SCL 22
  
  Wire.begin(SDA, SCL);                     // Start I2C communication
  //Wire.begin();                           // I2C deault ¡ojo! ESTA SENTENCIA TAMBIEN SERIA VALIDA
  Wire.setClock(400000);                    // 400kHz I2C clock. Comment this line if having compilation difficulties 
  Wire.beginTransmission(MPU_ADDRESS);      //Start communication with the MPU-6050.
  uint8_t error = Wire.endTransmission();   //End the transmission and register the exit status.
  while (error != 0) {                      //Stay in this loop because the MPU-6050 did not responde.
    Serial.print("**** ERROR MPU6050 : ");
    Serial.println(error);
    delay(1000);
    //LED->ON
  }
    
  #define LED_PIN 2 // Turn LED on during setup. 13->Arduino, 2->ESP32
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  
  setup_MPU6050_registers();
  
  calibration_MPU6050();
  
  digitalWrite(LED_PIN, LOW);
}

void calibration_MPU6050() {

  unsigned long duracion = millis();

  for (int cal_int = 0; cal_int < 2000 ; cal_int ++){ //STABILIZATION
    read_MPU6050_data();
    //gyro_x_cal += gyro_x;
    //gyro_y_cal += gyro_y;
    //gyro_z_cal += gyro_z;
    //acc_x_cal += acc_x;
    //acc_y_cal += acc_y;
    //acc_z_cal += acc_z;
    delayMicroseconds(SAMPLETIME-100); //delay of 4us to simulate the 250Hz program loop
  }

  for (int cal_int = 0; cal_int < 2000 ; cal_int ++){ //CALIBRATION
    read_MPU6050_data();
    gyro_x_cal += gyro_x;
    gyro_y_cal += gyro_y;
    gyro_z_cal += gyro_z;
    acc_x_cal += acc_x;
    acc_y_cal += acc_y;
    acc_z_cal += acc_z;
    delayMicroseconds(SAMPLETIME-100); //delay of 4us to simulate the 250Hz program loop
  }

  gyro_x_cal /= 2000;
  gyro_y_cal /= 2000;
  gyro_z_cal /= 2000;
  acc_x_cal /= 2000;
  acc_y_cal /= 2000;
  acc_z_cal /= 2000;

  Serial.println("Calibracion MPU6050:");
  Serial.print("calibracion GYRO:");Serial.print(gyro_x_cal);Serial.print(",");Serial.print(gyro_y_cal);Serial.print(",");Serial.println(gyro_z_cal);
  Serial.print("calibracion ACC:");Serial.print(acc_x_cal);Serial.print(",");Serial.print(acc_y_cal);Serial.print(",");Serial.println(acc_z_cal);  
  Serial.print("duracion calibracion: "); Serial.print((millis() - duracion)/1000); Serial.println(" segundos \n");
}

void setup_MPU6050_registers() {
  //https://www.invensense.com/products/motion-tracking/6-axis/mpu-6050/
  //DATASHEET:https://43zrtwysvxb2gf29r5o0athu-wpengine.netdna-ssl.com/wp-content/uploads/2015/02/MPU-6000-Datasheet1.pdf
  //REGISTERS:https://43zrtwysvxb2gf29r5o0athu-wpengine.netdna-ssl.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf
  
  //0x6B = PWR_MGMT_1
  //7             6      5      4   3                   2         1         0
  //Device-Reset  SLEEP  CYCLE  -   TemperatureDisable  CLKSEL 2  CLKSEL 1  CLKSEL 0
  //CLKSEL:
  //0  Internal 8MHz oscillator
  //1  PLL with X axis gyroscope reference
  //2  PLL with Y axis gyroscope reference
  //3  PLL with Z axis gyroscope reference
  //4  PLL with external 32.768kHz reference
  //5  PLL with external 19.2MHz reference
  //6  Reserved
  //7  Stops the clock and keeps the timing generator in reset
  
  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(0x6B);
  Wire.write(0x80); //TemperatureDisable
  Wire.endTransmission();
  delay(50);

  Wire.beginTransmission(MPU_ADDRESS); 
  Wire.write(0x6B);
  Wire.write(0x03); //PLL with Z axis gyroscope reference
  Wire.endTransmission();
  delay(50);
  
  //Wire.beginTransmission(MPU_ADDRESS); 
  //Wire.write(0x6B);
  //Wire.write(0x00); //Set the register bits as 00000000 to activate the gyro
  //Wire.endTransmission();
  //delay(50);

  //0x1B = GYRO_CONFIG
  //7     6     5       4         3         2 1 0
  //XG_ST YG_ST ZG_ST   FS_SEL 1  FS_SEL 0  - - -
  //FS_SEL:
  //0   ± 250 °/s
  //1   ± 500 °/s
  //2   ± 1000 °/s
  //3   ± 2000 °/s
  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(0x1B);
  Wire.write(0x08); //Set the register bits as 00001000 (500dps full scale)
  Wire.endTransmission(); 
  delay(50);

  //0x1C = ACCEL_CONFIG
  //7     6     5       4         3          2 1 0
  //XA_ST YA_ST ZA_ST   AFS_SEL 1 AFS_SEL 0  - - -
  //AFS_SEL:
  //0   ± 2g
  //1   ± 4g
  //2   ± 8g
  //3   ± 16g
  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(0x1C);
  Wire.write(0x10); //Set the register bits as 00010000 (+/- 8g full scale range)
  Wire.endTransmission();
  delay(50);

  //0x1A = CONFIGURATION
  //7 6 5               4               3               2           1           0
  //- - EXT_SYNC_SET 2  EXT_SYNC_SET 1  EXT_SYNC_SET 0  DLPF_CFG 2  DLPF_CFG 1  DLPF_CFG 0
  //EXT_SYNC_SET:
  //0   Input disabled
  //1   TEMP_OUT_L[0]
  //2   GYRO_XOUT_L[0]
  //3   GYRO_YOUT_L[0]
  //4   GYRO_ZOUT_L[0]
  //5   ACCEL_XOUT_L[0]
  //6   ACCEL_YOUT_L[0]
  //7   ACCEL_ZOUT_L[0]
  //DLPF_CFG (Digital Low Pass Frequency):
  //    Accelerometer             Gyroscope
  //    Bandwidth(Hz) Delay(ms)   Bandwidth(Hz) Delay(ms)   FS(kHz)
  //0   260            0.0        256            0.98       8
  //1   184            2.0        188            1.90       1
  //2    94            3.0         98            2.80       1
  //3    44            4.9         42            4.80       1
  //4    21            8.5         20            8.30       1
  //5    10           13.8         10           13.40       1
  //6     5           19.0          5           18.60       1
  //7   RESERVED                  RESERVED                  8
  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(0x1A);
  Wire.write(0x03); //Set the register bits as 03 = 00000011 (Set Digital Low Pass Filter to ~43Hz)
  Wire.endTransmission();
  delay(50);

  //uint8_t id;
  //i2cRead(MPU_ADDRESS, 0x75, 1, &id);
  //if (id == 0x68) 
  //    Serial.println("MPU6050 ID OK");
  //else 
  //    Serial.println("MPU6050 ID Failed");
}

void read_MPU6050_data() {
  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(0x3B); //Send the requested starting register
  Wire.endTransmission();
  
  Wire.requestFrom(MPU_ADDRESS,14);  //Request 14 bytes from the MPU6050
  while(Wire.available() < 14);
  acc_x = Wire.read()<<8|Wire.read();
  acc_y = Wire.read()<<8|Wire.read();
  acc_z = Wire.read()<<8|Wire.read();
  uint16_t temperature = Wire.read()<<8|Wire.read(); 
  gyro_x = Wire.read()<<8|Wire.read();
  gyro_y = Wire.read()<<8|Wire.read();
  gyro_z = Wire.read()<<8|Wire.read();
}

void i2cRead(uint8_t Address, uint8_t Register, uint8_t nBytes, uint8_t* Data) {
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.endTransmission();

  Wire.requestFrom(Address, nBytes);
  uint8_t index=0;
  while (Wire.available())
    Data[index++]=Wire.read();
}

void i2cWriteByte(uint8_t Address, uint8_t Register, uint8_t Data) {
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.write(Data);
  Wire.endTransmission();
}



void calculateAnglesMotion() {
  //Calibration
  gyro_x -= gyro_x_cal;
  gyro_y -= gyro_y_cal;
  gyro_z -= gyro_z_cal;
  //acc_x -= acc_x_cal;
  //acc_y -= acc_y_cal;
  //acc_z -= acc_z_cal;

  //Gyro angle calculations using integration => 1 / (FREQ / SSF_GYRO_1) where SSF_GYRO_1 = 65.5 or FS_SEL=1 (500º/s)
  angle_pitch += (float) gyro_x * integrator; //0.0000611;
  angle_roll += (float) gyro_y * integrator; //0.0000611; 

  angle_yaw += (float) gyro_z * integrator; //0.0000611; 
  //criterio YMFC
  if (angle_yaw < 0.0) 
        angle_yaw += 360.0;
  if (angle_yaw >= 360.0) 
        angle_yaw -= 360.0;

  //0.000001066 = 0.0000611 * (3.142(PI) / 180 degrees) why "sin" function is in radians
  angle_pitch -= angle_roll * sin(gyro_z * 0.000001066);
  angle_roll += angle_pitch * sin(gyro_z * 0.000001066);

  //Calculate the total accelerometer vector:√(X² + Y² + Z²)
  acc_total_vector = sqrt((acc_x*acc_x)+(acc_y*acc_y)+(acc_z*acc_z));
  //57.296 = 1 / (3.142 / 180) is the Arduino-Compatible "asin" function in radians
  //Prevent the "asin" function to produce a NaN
  if(abs(acc_y) < acc_total_vector){
    angle_pitch_acc = asin((float)acc_y/acc_total_vector)* 57.296;
  }
  if(abs(acc_x) < acc_total_vector){
    angle_roll_acc = asin((float)acc_x/acc_total_vector)* -57.296;
  }

  //Place the MPU6050 spirit level and note the values in the following two lines for calibration
  angle_pitch_acc_calibrado = angle_pitch_acc + pitch_calibrado;
  angle_roll_acc_calibrado = angle_roll_acc + roll_calibrado;

  if(set_gyro_angles){ //IMU is already started
      angle_pitch = angle_pitch * DRIFT_PITCH_ROLL + angle_pitch_acc_calibrado * (1.0-DRIFT_PITCH_ROLL); //Correct the drift of the gyro pitch angle with the accelerometer pitch angle
      angle_roll = angle_roll * DRIFT_PITCH_ROLL + angle_roll_acc_calibrado * (1.0-DRIFT_PITCH_ROLL); //Correct the drift of the gyro roll angle with the accelerometer roll angle
  }
  else{ //First Start = Reset Gyro Agles & IMU start flag
      angle_pitch = angle_pitch_acc_calibrado;  //Set the gyro pitch angle equal to the accelerometer pitch angle 
      angle_roll = angle_roll_acc_calibrado;  //Set the gyro roll angle equal to the accelerometer roll angle 
      angle_yaw = 0.0; //En el futuro este angulo sera el que marque el compas digital HMC5883L
      set_gyro_angles = true;
  }

  //To dampen the pitch and roll angles a complementary filter is used
  angle_pitch_output = HPF * angle_pitch_output + (1.0 - HPF) * angle_pitch;  //Take 90% of the output pitch value and add 10% of the raw pitch value
  angle_roll_output = HPF * angle_roll_output + (1.0 - HPF) * angle_roll;     //Take 90% of the output roll value and add 10% of the raw roll value

  angle_yaw_output = -1.0 * angle_yaw; //probamos sin DRIFT para que sea muy rapido pero con ¿demasiada? deriva
  //Traslado de datos para su visualizacion en los controles WEB
  if (angle_yaw_output > 180.0) {
        angle_yaw_output = angle_yaw_output - 360.0;
  } else if (angle_yaw_output <= -180.0)  {
        angle_yaw_output = angle_yaw_output + 360.0;
  }
  
  #if TRAZAS_ANGULOS
        Serial.print(F("angle_pitch_ouput:"));Serial.print(angle_pitch_output);Serial.print("\t");
        Serial.print(F("angular_motion_pitch:"));Serial.print(angular_motion_pitch);Serial.print("\t");
        Serial.print(F("angle_roll_ouput:"));Serial.print(angle_roll_output);Serial.print("\t");
  #endif
  
  #if PLOTTER_ANGULOS
        Serial.print(angle_pitch_output);Serial.print(",");
        Serial.print(angular_motion_pitch);Serial.print(",");
        Serial.print(angle_roll_output);Serial.print(",");
  #endif

  // Apply low-pass filter (10Hz cutoff frequency)
  // SSF_GYRO_1 = 65.5 => 1 deg/sec (check the datasheet of the MPU-6050 for more information)
  // Gyro pid input is deg/sec
  angular_motion_roll = LPF * angular_motion_roll + (1.0 - LPF) * ((float) gyro_y / 65.5);
  angular_motion_pitch = LPF * angular_motion_pitch + (1.0 - LPF) * ((float) gyro_x / 65.5);
  angular_motion_yaw = -1.0 * (LPF * angular_motion_yaw + (1.0 - LPF) * ((float) gyro_z / 65.5));

  #if TRAZAS_ANGULAR
        Serial.print(F("angular_motion_roll:"));Serial.print(angular_motion_roll);Serial.print("\t");
        //Serial.print(F("angle_yaw:"));Serial.print(angle_yaw);Serial.print("\t");
        Serial.print(F("angle_yaw_ouput:"));Serial.print(angle_yaw_output);Serial.print("\t");
        Serial.print(F("angular_motion_yaw:"));Serial.print(angular_motion_yaw);Serial.println("");        
  #endif
  
  #if PLOTTER_ANGULAR
        Serial.print(angular_motion_roll);Serial.print(",");
        //Serial.print(angle_yaw);Serial.print(",");
        Serial.print(angle_yaw_output);Serial.print(",");
        Serial.print(angular_motion_yaw);Serial.println("");
  #endif

  /*
  if (FiltroFusionSensores) {
        //TEST DE LA FUSION Y FILTRADO DE SENSORES

        Serial.print(angle_pitch_output); Serial.print(",");
        Serial.print( micros() );Serial.println("");
        
        //Serial.print(angle_roll+0.0); Serial.print(","); //offset visualizacion
        //Serial.print(angle_roll_acc); Serial.print(",");
        //Serial.print(angle_roll_acc_calibrado); Serial.print(",");
        //Serial.print(angle_roll_output); Serial.println("");
  }
  */
}

void resetGyroAngles() {
    angle_pitch = angle_pitch_acc_calibrado;  //Set the gyro pitch angle equal to the accelerometer pitch angle 
    angle_roll = angle_roll_acc_calibrado;  //Set the gyro roll angle equal to the accelerometer roll angle 
    angle_yaw = 0.0; //En el futuro este angulo sera el que marque el compas digital HMC5883L
}



void autoCalibracion_1() {
  contador_calibracion = 0;
  pitch_calibrado_temp = 0.0;
  roll_calibrado_temp = 0.0;
}

bool autoCalibracion_2() {
  if (contador_calibracion > 500) {
      pitch_calibrado_temp = pitch_calibrado_temp / 500.0;
      roll_calibrado_temp = roll_calibrado_temp / 500.0;
      return true;    
  } else {
      contador_calibracion++;
      pitch_calibrado_temp += angle_pitch_acc;
      roll_calibrado_temp += angle_roll_acc;
      return false;
  }
}
