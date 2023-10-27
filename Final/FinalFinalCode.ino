#include <MPU6050_tockn.h>
#include <Wire.h>
#include "CytronMotorDriver.h"
#include <SolarCalculator.h>
#include <TimeLib.h>
#include <DS1307RTC.h>

MPU6050 mpu6050(Wire);
CytronMD motor1(PWM_DIR, 7, 8);
CytronMD motor2(PWM_DIR, 6, 3);

bool yawCalibrated = false;
bool pitchCalibrated = false;
int tolerance = 1;

double latitude = 26.738180853550922;
double longitude = 94.21081269176345;
double utc_offset = 5.5;
int interval = 120;  // 2 minutes interval in seconds
time_t lastAlignmentTime = 0;
int currentAzimuth = 0; // Track the current azimuth
int currentPitch = 0; // Track the current pitch

void setup() {
  Serial.begin(19200);
  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
  setSyncProvider(RTC.get); // Set the RTC as the time provider
  lastAlignmentTime = now(); // Initialize the last alignment time
}

void loop() {
  mpu6050.update();
  time_t currentTime = now();

  // Calculate the solar position, in degrees
  double az, el;
  calcHorizontalCoordinates(currentTime - utc_offset * 3600L, latitude, longitude, az, el);


  if (!yawCalibrated) {
    int yaw = round(int(mpu6050.getAngleZ()));
    int yawError = round(az) - yaw;

    if (abs(yawError) < tolerance) {
      motor1.setSpeed(0);
      yawCalibrated = true;
      Serial.println("Yaw calibration completed.");
    } else {
      if (yawError < 0) {
        motor1.setSpeed(128);
      } else {
        motor1.setSpeed(-128);
      }
    }
    Serial.print("Yaw Angle: ");
    Serial.println(yaw);
  } else if (!pitchCalibrated) {
    int pitch = round(int(mpu6050.getAngleY()));
    int pitchError = round(el) - int(pitch);

    if (abs(pitchError) < tolerance) {
      motor2.setSpeed(0);
      pitchCalibrated = true;
      Serial.println("Pitch calibration completed.");
    } else {
      if (pitchError > 0) {
        motor2.setSpeed(-90); // Adjust the speed as needed
      } else {
        motor2.setSpeed(90); // Adjust the speed as needed
      }
    }
    Serial.print("Pitch Angle: ");
    Serial.println(pitch);
  }

  // Check if 2 minutes have passed since the last alignment
  if (currentTime - lastAlignmentTime >= interval) {
    int pitchChange = round(el)-currentPitch;
    int azimuthChange = round(az)-currentAzimuth;
    if(azimuthChange != 0){
     yawCalibrated = false;
    }
    if (pitchChange != 0) {
       pitchCalibrated = false;
    }  
    currentPitch = round(el);
    currentAzimuth = round(az);
    lastAlignmentTime = currentTime;
    
  }
}
