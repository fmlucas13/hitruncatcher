/*
  MPU6050 DMP6 for ROS + Motion LED Indicator
  LED will turn on when motion exceeds a threshold.

  - DMP handles complex motion processing.
  - LED indicates movement detected above a threshold.
*/

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <Servo.h>  // Include the Servo library

Servo myServo;  // Create a servo object

int servoPin = 9;  // You can use any PWM pin on the Mega (like 9, 10, 11, etc.)
// --- LED SETUP ---
const int ledPin = 13;  // LED connected to digital pin 13
const float motionThreshold = 1.0;

bool blink = true;
bool blink2 = true;
bool blink3 = true;
// Motion sensitivity threshold (adjustable)
// --- LED SETUP ---
const int ledPin2 = 12;              // LED connected to digital pin 12
const float motionThreshold2 = 3.0;  // Motion sensitivity threshold (adjustable)
// --- LED SETUP ---
const int ledPin3 = 11;              // LED connected to digital pin 11
const float motionThreshold3 = 5.0;  // Motion sensitivity threshold (adjustable)

// --- MPU6050 SETUP ---
MPU6050 mpu;
bool DMPReady = false;  // Flag for DMP initialization
uint8_t MPUIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint8_t FIFOBuffer[64];

// --- MPU6050 VARIABLES ---
Quaternion q;
VectorInt16 aa;       // Raw acceleration
VectorInt16 aaWorld;  // Gravity-compensated and rotated acceleration
VectorFloat gravity;
float ypr[3];  // Not used here, but for future orientation

void setup() {
  Wire.begin();
  Wire.setClock(400000);  // 400kHz I2C speed

  Serial.begin(115200);
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();

  Serial.println(F("Testing MPU6050 connection..."));
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed!");
    while (1)
      ;
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
  pinMode(ledPin2, OUTPUT);
  pinMode(ledPin3, OUTPUT);

  myServo.attach(servoPin);  // Connect servo signal wire to pin 9

  myServo.write(0);
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
    float az = aaWorld.z * mpu.get_acce_resolution() - 1.0;  // Remove gravity from z

    // Print acceleration for debugging
    Serial.print("ax: ");
    Serial.print(ax);
    Serial.print("\tay: ");
    Serial.print(ay);
    Serial.print("\taz: ");
    Serial.println(az);

    // --- MOTION DETECTION ---
    // If any axis exceeds threshold, turn LED on
    if (abs(ax) > motionThreshold || abs(ay) > motionThreshold || abs(az) > motionThreshold) {
      blink = false;
      if (blink == false) {
        myServo.write(-30);
      }
    }
    // --- MOTION DETECTION ---
    // If any axis exceeds threshold, turn LED on
    if (abs(ax) > motionThreshold2 || abs(ay) > motionThreshold2 || abs(az) > motionThreshold2) {
      blink2 = false;
      if (blink == false) {
        myServo.write(180);
        delay(500);
        myServo.write(90);
        delay(500);
        myServo.write(180);
        delay(500);
        myServo.write(90);
        delay(500);
        myServo.write(180);
        delay(500);
        myServo.write(90);
        delay(500);
        myServo.write(180);
        delay(500);
        myServo.write(90);
        delay(500);
        myServo.write(180);
        delay(500);
        myServo.write(90);
        delay(500);
        myServo.write(180);
        delay(500);
        myServo.write(90);
        delay(500);
        myServo.write(180);
        delay(500);
        myServo.write(90);
        delay(500);
        myServo.write(180);
        delay(500);
        myServo.write(90);
        delay(500);
        myServo.write(180);
        delay(500);
        myServo.write(90);
        delay(500);
        myServo.write(180);
        delay(500);
        myServo.write(90);
        delay(500);
        myServo.write(180);
        delay(500);
        myServo.write(90);
        delay(500);
        myServo.write(180);
        delay(500);
        myServo.write(90);
        delay(500);
        myServo.write(180);
        delay(500);
        myServo.write(90);
        delay(500);
        myServo.write(180);
        delay(500);
        myServo.write(90);
        delay(500);
        myServo.write(180);
        delay(500);
        myServo.write(90);
        delay(500);
        myServo.write(180);
        delay(500);
        myServo.write(90);
        delay(500);
        myServo.write(180);
        delay(500);
        myServo.write(90);
        delay(500);
        myServo.write(180);
        delay(500);
        myServo.write(90);
        delay(500);
        myServo.write(180);
        delay(500);
        myServo.write(90);
        delay(500);
        myServo.write(180);
        delay(500);
        myServo.write(90);
        delay(500);
        myServo.write(180);
        delay(500);
        myServo.write(90);
        delay(500);
        myServo.write(180);
        delay(500);
        myServo.write(90);
        delay(500);
        myServo.write(180);
        delay(500);
        myServo.write(90);
        delay(500);
        myServo.write(180);
        delay(500);
        myServo.write(90);
        delay(500);
        myServo.write(180);
        delay(500);
        myServo.write(90);
        delay(500);
        myServo.write(180);
        delay(500);
        myServo.write(90);
        delay(500);
        myServo.write(180);
        delay(500);
        myServo.write(90);
        delay(500);
        myServo.write(180);
        delay(500);
        myServo.write(90);
        delay(500);
        myServo.write(180);
        delay(500);
        myServo.write(90);
        delay(500);
        myServo.write(180);
        delay(500);
        myServo.write(90);
        delay(500);
        myServo.write(180);
        delay(500);
        myServo.write(90);
        delay(500);
        myServo.write(180);
        delay(500);
        myServo.write(90);
        delay(500);
        myServo.write(180);
        delay(500);
        myServo.write(90);
        delay(500);
        myServo.write(180);
        delay(500);
        myServo.write(90);
        delay(500);
        myServo.write(180);
        delay(500);
        myServo.write(90);
        delay(500);
        myServo.write(180);
        delay(500);
        myServo.write(90);
        delay(500);
        myServo.write(180);
        delay(500);
        myServo.write(90);
        delay(500);
        myServo.write(180);
        delay(500);
        myServo.write(90);
        delay(500);
        myServo.write(180);
        delay(500);
        myServo.write(90);
        delay(500);
        myServo.write(180);
        delay(500);
        myServo.write(90);
        delay(500);
        myServo.write(180);
        delay(500);
        myServo.write(90);
        delay(500);
        myServo.write(180);
        delay(500);
        myServo.write(90);
        delay(500);
        myServo.write(180);
        delay(500);
        myServo.write(90);
        delay(500);
        myServo.write(180);
        delay(500);
        myServo.write(90);
        delay(500);
      }
      // --- MOTION DETECTION ---
      // If any axis exceeds threshold, turn LED on
      if (abs(ax) > motionThreshold3 || abs(ay) > motionThreshold3 || abs(az) > motionThreshold3) {
        blink3 = false;
        if (blink == false) {
          myServo.write(180);
          delay(500);
          myServo.write(90);
          delay(500);
          myServo.write(180);
          delay(500);
          myServo.write(90);
          delay(500);
          myServo.write(180);
          delay(500);
          myServo.write(90);
          delay(500);
          myServo.write(180);
          delay(500);
          myServo.write(90);
          delay(500);
          myServo.write(180);
          delay(500);
          myServo.write(90);
          delay(500);
          myServo.write(180);
          delay(500);
          myServo.write(90);
          delay(500);
          myServo.write(180);
          delay(500);
          myServo.write(90);
          delay(500);
          myServo.write(180);
          delay(500);
          myServo.write(90);
          delay(500);
          myServo.write(180);
          delay(500);
          myServo.write(90);
          delay(500);
          myServo.write(180);
          delay(500);
          myServo.write(90);
          delay(500);
          myServo.write(180);
          delay(500);
          myServo.write(90);
          delay(500);
          myServo.write(180);
          delay(500);
          myServo.write(90);
          delay(500);
          myServo.write(180);
          delay(500);
          myServo.write(90);
          delay(500);
          myServo.write(180);
          delay(500);
          myServo.write(90);
          delay(500);
          myServo.write(180);
          delay(500);
          myServo.write(90);
          delay(500);
          myServo.write(180);
          delay(500);
          myServo.write(90);
          delay(500);
          myServo.write(180);
          delay(500);
          myServo.write(90);
          delay(500);
          myServo.write(180);
          delay(500);
          myServo.write(90);
          delay(500);
          myServo.write(180);
          delay(500);
          myServo.write(90);
          delay(500);
          myServo.write(180);
          delay(500);
          myServo.write(90);
          delay(500);
          myServo.write(180);
          delay(500);
          myServo.write(90);
          delay(500);
          myServo.write(180);
          delay(500);
          myServo.write(90);
          delay(500);
          myServo.write(180);
          delay(500);
          myServo.write(90);
          delay(500);
          myServo.write(180);
          delay(500);
          myServo.write(90);
          delay(500);
          myServo.write(180);
          delay(500);
          myServo.write(90);
          delay(500);
          myServo.write(180);
          delay(500);
          myServo.write(90);
          delay(500);
          myServo.write(180);
          delay(500);
          myServo.write(90);
          delay(500);
          myServo.write(180);
          delay(500);
          myServo.write(90);
          delay(500);
          myServo.write(180);
          delay(500);
          myServo.write(90);
          delay(500);
          myServo.write(180);
          delay(500);
          myServo.write(90);
          delay(500);
          myServo.write(180);
          delay(500);
          myServo.write(90);
          delay(500);
          myServo.write(180);
          delay(500);
          myServo.write(90);
          delay(500);
          myServo.write(180);
          delay(500);
          myServo.write(90);
          delay(500);
          myServo.write(180);
          delay(500);
          myServo.write(90);
          delay(500);
          myServo.write(180);
          delay(500);
          myServo.write(90);
          delay(500);
          myServo.write(180);
          delay(500);
          myServo.write(90);
          delay(500);
          myServo.write(180);
          delay(500);
          myServo.write(90);
          delay(500);
          myServo.write(180);
          delay(500);
          myServo.write(90);
          delay(500);
          myServo.write(180);
          delay(500);
          myServo.write(90);
          delay(500);
          myServo.write(180);
          delay(500);
          myServo.write(90);
          delay(500);
          myServo.write(180);
          delay(500);
          myServo.write(90);
          delay(500);
          myServo.write(180);
          delay(500);
          myServo.write(90);
          delay(500);
          myServo.write(180);
          delay(500);
          myServo.write(90);
          delay(500);
          myServo.write(180);
          delay(500);
          myServo.write(90);
          delay(500);
        }
        if (blink == true) {
          digitalWrite(ledPin, LOW);
          delay(30);

          digitalWrite(ledPin, HIGH);
        }
        if (blink2 == true) {
          digitalWrite(ledPin2, LOW);
          delay(30);

          digitalWrite(ledPin2, HIGH);
        }
        if (blink3 == true) {
          digitalWrite(ledPin3, LOW);
          delay(30);

          digitalWrite(ledPin3, HIGH);
        }
      }
    }
  }
}