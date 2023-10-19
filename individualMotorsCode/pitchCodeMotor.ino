#include <MPU6050_tockn.h>
#include <Wire.h>


MPU6050 mpu6050(Wire);
// Define the motor pins
int enB = 9; // Enable B (motor 2, up/down)
int in1 = 3; // Input 1 (motor 2, up)
int in2 = 6; // Input 2 (motor 2, down)
// Define the required yaw angle
int requirePitchAngle = 30;
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
  float pitch = -mpu6050.getAngleY();
  float yaw = mpu6050.getAngleZ();

  // Calculate the error between the required yaw angle and the current yaw angle
  int pitchError =  requirePitchAngle - int(pitch);

  // Calculate the motor speed
  int motorSpeed = abs(pitchError) * 10.0;

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
