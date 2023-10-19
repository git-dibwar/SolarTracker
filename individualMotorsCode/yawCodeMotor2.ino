#include <MPU6050_tockn.h>
#include <Wire.h>
#include "CytronMotorDriver.h"

MPU6050 mpu6050(Wire);
CytronMD motor1(PWM_DIR, 7, 8); // PWM 2 = Pin 7, DIR 2 = Pin 8.

// Define the required yaw angle
int requiredYawAngle = 20;
bool motorStopped = false;

void setup() {
  Serial.begin(19200);
  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
}

void loop() {
  mpu6050.update();

  // Extract roll, pitch, and yaw angles
  int yaw = int(mpu6050.getAngleZ());

  // Calculate the error between the required yaw angle and the current yaw angle
  int error = requiredYawAngle - yaw;

  // Calculate the motor speed
  int setSpeed = error * 10;

  if (!motorStopped) {
    if (abs(error) < 1) { // Adjust the threshold for stopping the motor
      motor1.setSpeed(0);
      motorStopped = true;
    } else {
      if (error < 0) {
        motor1.setSpeed(128);
      } else {
        motor1.setSpeed(-128);
      }
    }
  }

  // Print the current yaw angle
  Serial.println("Yaw Angle: " + String(yaw));

  delay(500);
}
