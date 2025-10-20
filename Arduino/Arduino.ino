/*
  MPU6050 DMP6 for ROS + Motion LED Indicator
  LED will turn on when motion exceeds a threshold.

  - DMP handles complex motion processing.
  - LED indicates movement detected above a threshold.
*/

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

// --- LED SETUP ---
const int ledPin = 13;       // LED connected to digital pin 13
const float motionThreshold = 3.0;  // Motion sensitivity threshold (adjustable)

// --- MPU6050 SETUP ---
MPU6050 mpu;
bool DMPReady = false;  // Flag for DMP initialization
uint8_t MPUIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint8_t FIFOBuffer[64];

// --- MPU6050 VARIABLES ---
Quaternion q;
VectorInt16 aa;         // Raw acceleration
VectorInt16 aaWorld;    // Gravity-compensated and rotated acceleration
VectorFloat gravity;
float ypr[3];           // Not used here, but for future orientation

void setup() {
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C speed

  Serial.begin(115200);
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();

  Serial.println(F("Testing MPU6050 connection..."));
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed!");
    while (1);
  } else {
    Serial.println("MPU6050 connection successful!");
  }

  // DMP Initialization
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // Offsets (can be auto-calibrated or manually tuned)
  mpu.setXGyroOffset(0);
  mpu.setYGyroOffset(0);
  mpu.setZGyroOffset(0);
  mpu.setXAccelOffset(0);
  mpu.setYAccelOffset(0);
  mpu.setZAccelOffset(0);

  if (devStatus == 0) {
    mpu.CalibrateAccel();
    mpu.CalibrateGyro();
    Serial.println("Active offsets:");
    mpu.PrintActiveOffsets();

    mpu.setDMPEnabled(true);
    MPUIntStatus = mpu.getIntStatus();
    DMPReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
    Serial.println(F("DMP ready!"));
  }

  // LED pin setup
  pinMode(ledPin, OUTPUT);
      digitalWrite(ledPin, LOW); 
}

void loop() {
  if (!DMPReady) return;

  if (mpu.dmpGetCurrentFIFOPacket(FIFOBuffer)) {
    // Get gravity-compensated acceleration
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetAccel(&aa, FIFOBuffer);
    mpu.dmpConvertToWorldFrame(&aaWorld, &aa, &q);

    // Convert raw acceleration to g's (gravity units)
    float ax = aaWorld.x * mpu.get_acce_resolution();
    float ay = aaWorld.y * mpu.get_acce_resolution();
    float az = aaWorld.z * mpu.get_acce_resolution() - 1.0; // Remove gravity from z

    // Print acceleration for debugging
    Serial.print("ax: "); Serial.print(ax);
    Serial.print("\tay: "); Serial.print(ay);
    Serial.print("\taz: "); Serial.println(az);


    // --- MOTION DETECTION ---
    // If any axis exceeds threshold, turn LED on
    if (abs(ax) > motionThreshold || abs(ay) > motionThreshold || abs(az) > motionThreshold) {
      digitalWrite(ledPin, HIGH); // Motion detected
}
//  if (abs(ax) < - motionThreshold || abs(ay) < - motionThreshold || abs(az) < - motionThreshold) {
//       digitalWrite(ledPin, HIGH); // Motion detected
// }
    delay(200); // small delay for stability
  }
}
