#include <MPU6050_tockn.h>
#include <Wire.h>


MPU6050 mpu6050(Wire);
// Define the motor pins
int enB = 9; // Enable A (motor 1, left/right)
int in1 = 3; // Input 1 (motor 1, left)
int in2 = 6; // Input 2 (motor 1, right)
// Define the required yaw angle
int requirePitchAngle = -30;
void setup() {
  Serial.begin(19200);
  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);



  // Set the motor pins as outputs
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);

  // Start the motor
  analogWrite(enB, 255);

}

void loop() {
  mpu6050.update();

  // Extract roll, pitch, and yaw angles
  float roll = mpu6050.getAngleX();
  int pitch = int(mpu6050.getAngleY());
  int yaw = int(mpu6050.getAngleZ());

  // Calculate the error between the required yaw angle and the current yaw angle
  int pitchError = requirePitchAngle - pitch;

  // Calculate the motor speed
  int motorSpeed = pitchError * 10.0;

  // Set the motor direction
  if (pitchError > 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  } else {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  }

  Serial.print("Pitch: " + String(pitch));
  Serial.println();
  // Set the motor speed
  analogWrite(enB, motorSpeed);

  // Stop the motor when the required yaw angle is reached
  if (abs(pitchError) < 2.0) {
    analogWrite(enB, 0);
  }



  delay(500);
}
