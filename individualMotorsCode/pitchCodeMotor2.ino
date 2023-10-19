#include <MPU6050_tockn.h>
#include <Wire.h>
#include "CytronMotorDriver.h"

MPU6050 mpu6050(Wire);
CytronMD motor1(PWM_DIR, 6, 3);  // PWM 1 = Pin 3, DIR 1 = Pin 6.

// Define the required pitch angle
int requiredPitchAngle = -17;
bool motorStopped = false;

void setup() {
  Serial.begin(19200);
  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
}

void loop() {
  mpu6050.update();

  // Extract pitch angle
  float pitch = mpu6050.getAngleY();

  // Calculate the error between the required pitch angle and the current pitch angle
  int pitchError = requiredPitchAngle - int(pitch);

  // Calculate the motor speed
  int motorSpeed = abs(pitchError) * 10;

  if (!motorStopped) {
    if (abs(pitchError) < 1) { // Adjust the threshold for stopping the motor
      motor1.setSpeed(0);
      motorStopped = true;
    } else {
      if (pitchError > 0) {
        motor1.setSpeed(-128);  // Adjust the speed as needed
      } else {
        motor1.setSpeed(128);  // Adjust the speed as needed
      }
    }
  }

  // Print the current pitch angle
  Serial.println("Pitch Angle: " + String(pitch));

  delay(500);
}
