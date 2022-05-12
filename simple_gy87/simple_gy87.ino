#include "I2Cdev.h"
#include "MPU6050.h"
#include "HMC5883L_Simple.h"

MPU6050 accelgyro;
HMC5883L_Simple Compass;

int16_t ax, ay, az;
int16_t gx, gy, gz;

#define LED_PIN 13
bool blinkState = false;

void setup() {
  Serial.begin(9600);
  Wire.begin();

  // initialize devices
  Serial.println("Initializing I2C devices...");
  // initialize mpu6050
  accelgyro.initialize();
  //Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
  accelgyro.setI2CBypassEnabled(true); // set bypass mode for gateway to hmc5883L


  // initialize hmc5883l
  Compass.SetDeclination(32, 59, 'E'); //http://www.magnetic-declination.com/
  Compass.SetSamplingMode(COMPASS_SINGLE); // Single = Read only when requested
  Compass.SetScale(COMPASS_SCALE_130); // Sensitivity
  Compass.SetOrientation(COMPASS_HORIZONTAL_X_NORTH); //Mounting orientation

  // configure Arduino LED for checking activity
  pinMode(LED_PIN, OUTPUT);
}

void loop() {

  // read raw accel/gyro measurements from device
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // display tab-separated accel/gyro x/y/z values
  Serial.print("a/g:\t");
  Serial.print(ax); Serial.print("\t");
  Serial.print(ay); Serial.print("\t");
  Serial.print(az); Serial.print("\t");
  Serial.print(gx); Serial.print("\t");
  Serial.print(gy); Serial.print("\t");
  Serial.println(gz);
  
  // Accelerometers sensitivity:
  // -/+2g = 16384  LSB/g
  float xGf = (float)ax / (float)16384;
  float yGf = (float)ay / (float)16384;
  float zGf = (float)az / (float)16384;
  
  // display tab-separated G-forces values
  Serial.print("Acc:\t");
  Serial.print(xGf); Serial.print("\t");
  Serial.print(yGf); Serial.print("\t");
  Serial.println(zGf);

  // Convert accelerations to angle
  float roll = 180 * atan(xGf / sqrt((yGf * yGf) + (zGf * zGf))) / M_PI;
  float pitch = 180 * atan(yGf / sqrt((xGf * xGf) + (zGf * zGf))) / M_PI;
  //float yaw = 180 * atan(sqrt((xGf * xGf) + (yGf * yGf)) / zGf) / M_PI;

  // display tab-separated G-forces values
  Serial.print("Ang:\t");
  Serial.print(roll); Serial.print("\t");
  Serial.print(pitch); Serial.print("\t");
//  Serial.println(yaw);
  
  float heading = Compass.GetHeadingDegrees();
  //Serial.print("Head: \t");
  Serial.println( heading );

  // blink LED to indicate activity
  blinkState = !blinkState;
  digitalWrite(LED_PIN, blinkState);

  delay(500);
}
