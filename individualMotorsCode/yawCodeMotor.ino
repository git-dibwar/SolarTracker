#include <MPU6050_tockn.h>
#include <Wire.h>


MPU6050 mpu6050(Wire);
// Define the motor pins
int enA = 10; // Enable A (motor 1, left/right)
int in1 = 8; // Input 1 (motor 1, left)
int in2 = 7; // Input 2 (motor 1, right)
// Define the required yaw angle
int requiredYawAngle = -90;
void setup() {
  Serial.begin(19200);
  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);



  // Set the motor pins as outputs
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);

  // Start the motor
  analogWrite(enA, 255);

}

void loop() {
  mpu6050.update();

  // Extract roll, pitch, and yaw angles
  float roll = mpu6050.getAngleX();
  float pitch = mpu6050.getAngleY();
  int yaw = int(mpu6050.getAngleZ());

  // Calculate the error between the required yaw angle and the current yaw angle
  int error = requiredYawAngle - yaw;

  // Calculate the motor speed
  int motorSpeed = error * 10.0;

  // Set the motor direction
  if (error > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  }

  Serial.print("Azimuth: " + String(yaw));
  Serial.println();
  // Set the motor speed
  analogWrite(enA, motorSpeed);

  // Stop the motor when the required yaw angle is reached
  if (abs(error) < 1.0) {
    analogWrite(enA, 0);
  }



  delay(500);
}
