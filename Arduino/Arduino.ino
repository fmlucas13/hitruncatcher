#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Servo.h>
Adafruit_MPU6050 mpu;
#define LED_PIN 13
#define SERVO_PIN 12

Servo myServo;

bool triggered = false;
const float motionthershold = 2.0;
void setup(void) {
  Serial.begin(115200);
  while (!Serial)
    delay(10);  // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit MPU6050 test!");

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");
  pinMode(LED_PIN, OUTPUT);
  myServo.attach(SERVO_PIN);
  myServo.write(0); 
  //setupt motion detection
  mpu.setHighPassFilter(MPU6050_HIGHPASS_0_63_HZ);
  mpu.setMotionDetectionThreshold(1);
  mpu.setMotionDetectionDuration(20);
  mpu.setInterruptPinLatch(true);  // Keep it latched.  Will turn off when reinitialized.
  mpu.setInterruptPinPolarity(true);
  mpu.setMotionInterrupt(true);

  Serial.println("");
  delay(100);
}

void loop() {
  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  /* Print out the values */
  Serial.print("AccelX:");
  Serial.print(a.acceleration.x);
  Serial.print(",");
  Serial.print("\t\tAccelY:");
  Serial.print(a.acceleration.y);
  Serial.print(",");
  Serial.print("\t\tAccelZ:");
  Serial.print(a.acceleration.z);
  Serial.print(", ");
  Serial.print("\t\tGyroX:");
  Serial.print(g.gyro.x);
  Serial.print(",");
  Serial.print("\t\tGyroY:");
  Serial.print(g.gyro.y);
  Serial.print(",");
  Serial.print("\t\tGyroZ:");
  Serial.print(g.gyro.z);


  float x = a.acceleration.x;
  float y = a.acceleration.y;
  float z = a.acceleration.z;

  // Calculate total g
  float tg = (sqrt(x * x + y * y + z * z) - 9.81) / 9.81;
  Serial.print("\t\tTotal Gs: ");
  Serial.print(tg);
  Serial.println("");

  if (tg > motionthershold) {
    if (!triggered) {
      triggered = true;
      myServo.write(180);
      digitalWrite(LED_PIN, HIGH);  // Solid LED
    }
  } else {
    if (!triggered) {
      digitalWrite(LED_PIN, HIGH);
      delay(50);
      digitalWrite(LED_PIN, LOW);
      delay(50);
    }
  }
}