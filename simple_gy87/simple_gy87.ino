#include "I2Cdev.h"
#include "MPU6050.h"
#include "HMC5883L_Simple.h"

MPU6050 accelgyro;
HMC5883L_Simple Compass;

int16_t ax, ay, az; // accelerometer raw data
int16_t gx, gy, gz; //gyro raw data
double gyroXdeg = 0; double gyroYdeg = 0; double gyroZdeg = 0;

// Kalman filter parameter
float SensorData, KalmanFilterData;
float Xt, Xt_update, Xt_prev;
float Pt, Pt_update, Pt_prev = 1;
float Kt; float R = 1.5; float Q = 0.5;

// MPU6050 offsets
int16_t ax_offset = 1121; int16_t ay_offset = -1342; int16_t az_offset = 1263;
int16_t gx_offset = -6499; int16_t gy_offset = -19965; int16_t gz_offset = 5044;;

unsigned long currentTime, pastTime, difTime;
bool firstTime = true;

#define LED_PIN 13
bool blinkState = false;

// For Debug purpose
bool showDebug = false;
bool includeYaw = false;
bool showRawMPU = false;
bool showRawG = false;
bool showAccAng = true;
bool showGyrAng = false;
bool showCompAng = false;
bool showKalmanAng = true;
void setup() {
  Serial.begin(9600);
  Wire.begin();

  // initialize devices
  if (showDebug) Serial.println("Initializing I2C devices...");
  // initialize MPU6050
  accelgyro.initialize();
  // set MPU6050 offsets
  accelgyro.setXAccelOffset(ax_offset);
  accelgyro.setYAccelOffset(ay_offset);
  accelgyro.setZAccelOffset(az_offset);
  accelgyro.setXGyroOffset(gx_offset);
  accelgyro.setYGyroOffset(gy_offset);
  accelgyro.setZGyroOffset(gz_offset);
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
  // get timestamp
  if (firstTime) { // if the first run, adjust to set the first pastTime
    currentTime = millis() / 1000;
    delay(50);
    pastTime = currentTime;
    firstTime = false;
  }
  currentTime = millis() / 1000;
  difTime = currentTime - pastTime; //0.01;
  pastTime = currentTime;

  // read raw accel/gyro measurements from device
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // display tab-separated accel/gyro x/y/z values
  if (showRawMPU) {
    Serial.print("a/g:\t");
    Serial.print(ax);
    Serial.print("\t");
    Serial.print(ay);
    Serial.print("\t");
    Serial.print(az);
    Serial.print("\t");
    Serial.print(gx);
    Serial.print("\t");
    Serial.print(gy);
    Serial.print("\t");
    Serial.println(gz);
  }

  // Accelerometers sensitivity:
  // -/+2g = 16384  LSB/g
  float xGf = (float)ax / (float)16384;
  float yGf = (float)ay / (float)16384;
  float zGf = (float)az / (float)16384;

  // display tab-separated G-forces values
  if (showRawG) {
    Serial.print("Acc:\t");
    Serial.print(xGf); Serial.print("\t"); Serial.print(yGf); Serial.print("\t"); Serial.println(zGf);
  }
  // Convert accelerations to angle
  float roll = 180 * atan(xGf / sqrt((yGf * yGf) + (zGf * zGf))) / M_PI;
  float pitch = 180 * atan(yGf / sqrt((xGf * xGf) + (zGf * zGf))) / M_PI;
  float yaw = 180 * atan(sqrt((xGf * xGf) + (yGf * yGf)) / zGf) / M_PI;

  // display Accel Angle
  if (showAccAng) {
    Serial.print("Roll:"); Serial.print(roll);
    Serial.print("\tPitch:"); Serial.print(pitch);
    if (includeYaw) {
      Serial.print("\tYaw:");
      Serial.print(yaw);
    }
  }

  //Convert Gyro raw data to rotation rate
  double gyroXrate = gx / 131.0; // Convert to deg/s
  double gyroYrate = gy / 131.0; // Convert to deg/s
  double gyroZrate = gz / 131.0; // Convert to deg/s
  //Integrate Gyro rotation rate to degree
  gyroXdeg += (gyroXrate * difTime) + 0.438; // Convert to degree // compensate drift
  gyroYdeg += (gyroYrate * difTime) + 0.005; // Convert to degree
  if (includeYaw) {
    gyroZdeg += gyroZrate * difTime; // Convert to degree
  }

  // display Gyro Angle
  if (showGyrAng) {
    Serial.print("\tRollGyro:"); Serial.print(gyroXdeg);
    Serial.print("\tPitchGyro:"); Serial.print(gyroYdeg);
    if (includeYaw) {
      Serial.print("\tYawGyro:");
      Serial.print(gyroZdeg);
    }
  }

  float heading = Compass.GetHeadingDegrees();
  // display Heading
  if (showDebug) {
    Serial.print("Heading:");
    Serial.println(heading);
  }

  // Complementary Filter
  float rollComp = 0.98 * (gyroXdeg) + 0.02 * roll;
  float pitchComp = 0.98 * (gyroYdeg) + 0.02 * pitch;
  if (includeYaw) {
    float yawComp = 0.98 * (gyroZdeg) + 0.02 * yaw;
  }

  // display Gyro Angle
  if (showCompAng) {
    Serial.print("\tRollComp:"); Serial.print(rollComp);
    Serial.print("\tPitchComp:"); Serial.print(rollComp);
    if (includeYaw) {
      Serial.print("\tYawComp:");
      Serial.print(rollComp);
    }
  }
  // Kalman Filter
  SensorData = roll;
  Xt_update = Xt_prev;
  Pt_update = Pt_prev + Q;
  Kt = Pt_update / (Pt_update + R);
  Xt = Xt_update + (Kt * (SensorData - Xt_update));
  Pt = (1 - Kt) * Pt_update;
  Xt_prev = Xt;
  Pt_prev = Pt;
  KalmanFilterData = Xt;
  if (showKalmanAng) {
    Serial.print("\tRollKalman:"); Serial.print(KalmanFilterData);
  }

  Serial.print("\n");
  // blink LED to indicate activity
  blinkState = !blinkState;
  digitalWrite(LED_PIN, blinkState);
  delay(50);
}
