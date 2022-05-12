#include <Wire.h>
long accelX, accelY, accelZ;
float gForceX, gForceY, gForceZ;

long gyroX, gyroY, gyroZ;
float rotX, rotY, rotZ;

// GY-87 I2C devices address:
// HMC5883L found at address 0x1E or 0b00011110
// MPU6050 found at address 0x68 or 0b01101000
// BMP180 found at address 0x77 or 0b01110111

#define MPU6050_ADDRESS 0b1101000
#define HMC5883L_ADDRESS 0b11110

void setup() {
  Serial.begin(9600);
  Wire.begin();
  setupMPU();
  setupHMC();
}


void loop() {
  recordAccelRegisters();
  recordGyroRegisters();
  printData();
  delay(100);
}

void setupMPU(){
  Wire.beginTransmission(MPU6050_ADDRESS); //This is the I2C address of the MPU (b1101000/b1101001 for AC0 low/high datasheet sec. 9.2)
  Wire.write(0x6B); //Accessing the register 6B - Power Management
  Wire.write(0b00000000); //Setting SLEEP register to 0.
  Wire.endTransmission();  
  Wire.beginTransmission(MPU6050_ADDRESS); 
  Wire.write(0x1B); //Accessing the register 1B - Gyroscope Configuration
  Wire.write(0x00000000); //Setting the gyro to full scale +/- 250deg./s 
  Wire.endTransmission(); 
  Wire.beginTransmission(MPU6050_ADDRESS); 
  Wire.write(0x1C); //Accessing the register 1C - Acccelerometer Configuration
  Wire.write(0b00000000); //Setting the accel to +/- 2g
  Wire.endTransmission(); 
}

void setupHMC(){
  Wire.beginTransmission(MPU6050_ADDRESS); 
  Wire.write(0x6A); //Accessing the register 6A - User Control 
  Wire.write(0b00000000); //Disabling I2C master mode
  Wire.endTransmission();
  Wire.beginTransmission(MPU6050_ADDRESS); 
  Wire.write(0x37); //Accessing Int
  Wire.write(0x02); //Enabling I2C master bypass mode
  Wire.endTransmission();
  Wire.beginTransmission(HMC5883L_ADDRESS); 
  Wire.write(0x00); //
  Wire.write(0x18); //set sampling rate to 75Hz
  Wire.endTransmission();
  Wire.beginTransmission(HMC5883L_ADDRESS); 
  Wire.write(0x01); //
  Wire.write(0x60); //full scale +- 2.5 gauss
  Wire.endTransmission();
  Wire.beginTransmission(HMC5883L_ADDRESS); 
  Wire.write(0x02); //
  Wire.write(0x00); //Continuous sampling
  Wire.endTransmission();
  Wire.beginTransmission(MPU6050_ADDRESS); 
  Wire.write(0x37); //Accessing Int
  Wire.write(0x00); //Disabling I2C master bypass mode
  Wire.endTransmission();
  Wire.beginTransmission(MPU6050_ADDRESS); 
  Wire.write(0x6A); //Accessing Int
  Wire.write(0x20); //Enabling I2C master mode
  Wire.endTransmission();
  //Pass HMC to MPU
  Wire.beginTransmission(MPU6050_ADDRESS); 
  Wire.write(0x25); //
  Wire.write(HMC5883L_ADDRESS | 0x80); //set slave 0 i2c address, read mode
  Wire.endTransmission();
  Wire.beginTransmission(MPU6050_ADDRESS); 
  Wire.write(0x26); // 
  Wire.write(0x03); // set slave 0 register = 0x03 (x axis)
  Wire.endTransmission();
  Wire.beginTransmission(MPU6050_ADDRESS); 
  Wire.write(0x27); // 
  Wire.write(6 | 0x80); // set slave 0 transfer size = 6, enabled
  Wire.endTransmission();
  Wire.beginTransmission(MPU6050_ADDRESS); 
  Wire.write(0x67); // 
  Wire.write(1); // enable slave 0 delay
  Wire.endTransmission();
}

void recordAccelRegisters() {
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x3B); //Starting register for Accel Readings
  Wire.endTransmission();
  Wire.requestFrom(0b1101000,6); //Request Accel Registers (3B - 40)
  while(Wire.available() < 6);
  accelX = Wire.read()<<8|Wire.read(); //Store first two bytes into accelX
  accelY = Wire.read()<<8|Wire.read(); //Store middle two bytes into accelY
  accelZ = Wire.read()<<8|Wire.read(); //Store last two bytes into accelZ
  processAccelData();
}

void processAccelData(){
  gForceX = accelX / 16384.0;
  gForceY = accelY / 16384.0; 
  gForceZ = accelZ / 16384.0;
}

void recordGyroRegisters() {
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x43); //Starting register for Gyro Readings
  Wire.endTransmission();
  Wire.requestFrom(0b1101000,6); //Request Gyro Registers (43 - 48)
  while(Wire.available() < 6);
  gyroX = Wire.read()<<8|Wire.read(); //Store first two bytes into accelX
  gyroY = Wire.read()<<8|Wire.read(); //Store middle two bytes into accelY
  gyroZ = Wire.read()<<8|Wire.read(); //Store last two bytes into accelZ
  processGyroData();
}

void processGyroData() {
  rotX = gyroX / 131.0;
  rotY = gyroY / 131.0; 
  rotZ = gyroZ / 131.0;
}

void printData() {
  Serial.print("Gyro (deg)");
  Serial.print(" X=");
  Serial.print(rotX);
  Serial.print(" Y=");
  Serial.print(rotY);
  Serial.print(" Z=");
  Serial.print(rotZ);
  Serial.print(" Accel (g)");
  Serial.print(" X=");
  Serial.print(gForceX);
  Serial.print(" Y=");
  Serial.print(gForceY);
  Serial.print(" Z=");
  Serial.println(gForceZ);
}
