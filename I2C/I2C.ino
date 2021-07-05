
#include <Wire.h>

// #define ADDRESS_SENSOR 0x68 // direccion del sensor MPU6050
// #define ADDRESS_SENSOR 0x77 // direccion del sensor BMP180
// #define ADDRESS_SENSOR 0x0D // direccion del sensor HMC5883L
// #define ADDRESS_SENSOR 0x77 // direccion del sensor MS5611

#define SDA 21 //BUS PRINCIPAL
#define SCL 22
#define SDA1 4 //BUS SECUNDARIO
#define SCL1 5

byte error, address;
int devices;

void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }
  Serial.println("\nI2C Scanner\n");
  
  Wire.begin(SDA, SCL);
  //Wire.setClock(400000); // 400kHz I2C clock (Comment this line if having compilation difficulties)
  
  Wire1.begin(SDA1, SCL1);
  //Wire1.setClock(400000); // 400kHz I2C clock (Comment this line if having compilation difficulties)
}

void loop() {
  bus();
  bus1();   
  delay(5000);          
}

void bus() {
  Serial.println("Scanning BUS ...");
  devices = 0;
  
  for(address = 1; address < 127; address++ ) {
      Wire.beginTransmission(address);
      error = Wire.endTransmission();
      if (error == 0) {
          Serial.print("I2C-BUS device found at address 0x");
          printAddress();
          devices++;
      }
      else if (error==4) {
          Serial.print("BUS - Unknown error at address 0x");
          printAddress();
      }    
  }
  
  if (devices == 0) {
    Serial.println("No I2C-BUS devices found\n");
  } else {
    Serial.println("done\n");
  }
}

void bus1() {
  Serial.println("Scanning BUS1 ...");
  devices = 0;
  
  for(address = 1; address < 127; address++ ) {
      Wire1.beginTransmission(address);
      error = Wire1.endTransmission();
      if (error == 0) {
          Serial.print("I2C-BUS1 device found at address 0x");
          printAddress();
          devices++;
      }
      else if (error==4) {
          Serial.print("BUS1 - Unknown error at address 0x");
          printAddress();
      }    
  }
  
  if (devices == 0) {
    Serial.println("No I2C-BUS1 devices found\n");
  } else {
    Serial.println("done\n");
  }
}

void printAddress () {
    if (address<16) {
        Serial.print("0");
    }
    Serial.println(address,HEX);
}
