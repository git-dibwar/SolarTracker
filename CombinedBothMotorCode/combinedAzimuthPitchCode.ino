#include <MPU6050_tockn.h>
#include <Wire.h>

MPU6050 mpu6050(Wire);

// Define the motor pins for pitch
int enPitch = 9; // Enable A (motor 1, left/right)
int in1Pitch = 3; // Input 1 (motor 1, left)
int in2Pitch = 6; // Input 2 (motor 1, right)

// Define the motor pins for yaw
int enYaw = 10; // Enable B (motor 2, left/right)
int in1Yaw = 8; // Input 1 (motor 2, left)
int in2Yaw = 7; // Input 2 (motor 2, right)

// Define the required pitch and yaw angles
int requiredPitchAngle = 20;
int requiredYawAngle = 20;

void setup() {
  Serial.begin(19200);
  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);

  // Set the motor pins as outputs
  pinMode(enPitch, OUTPUT);
  pinMode(in1Pitch, OUTPUT);
  pinMode(in2Pitch, OUTPUT);

  pinMode(enYaw, OUTPUT);
  pinMode(in1Yaw, OUTPUT);
  pinMode(in2Yaw, OUTPUT);

  // Start both motors
  analogWrite(enPitch, 255);
  analogWrite(enYaw, 255);
}

void loop() {
  mpu6050.update();

  // Extract roll, pitch, and yaw angles
  float roll = mpu6050.getAngleX();
  float pitch = float(-mpu6050.getAngleY());
  float yaw = float(mpu6050.getAngleZ());

  // Calculate the pitch error
  int pitchError = requiredPitchAngle - int(pitch);
  int pitchMotorSpeed = pitchError * 10.0;

  // Calculate the yaw error
  int yawError = requiredYawAngle - yaw;
  int yawMotorSpeed = yawError * 10.0;

  // Set the motor direction for pitch
  if (pitchError < 0) {
    digitalWrite(in1Pitch, LOW);
    digitalWrite(in2Pitch, HIGH);
  } else {
    digitalWrite(in1Pitch, HIGH);
    digitalWrite(in2Pitch, LOW);
  }

  // Set the motor direction for yaw
  if (yawError > 0) {
    digitalWrite(in1Yaw, HIGH);
    digitalWrite(in2Yaw, LOW);
  } else {
    digitalWrite(in1Yaw, LOW);
    digitalWrite(in2Yaw, HIGH);
  }
   Serial.print("Azimuth: " + String(yaw)+" Pitch: " + String(pitch));
  Serial.println();
  
  // Set the motor speed for pitch
  analogWrite(enPitch, pitchMotorSpeed);

  // Set the motor speed for yaw
  analogWrite(enYaw, yawMotorSpeed);

  // Stop both motors when the required pitch and yaw angles are reached
  if (abs(pitchError) < 2.0 && abs(yawError) < 1.0) {
    analogWrite(enPitch, 0);
    analogWrite(enYaw, 0);
  }

  delay(500);
}
