#include <MPU6050_tockn.h>
#include <Wire.h>
#include "CytronMotorDriver.h"
#include <SolarCalculator.h>
#include <TimeLib.h>

//==================================================================================


MPU6050 mpu6050(Wire);
CytronMD motor1(PWM_DIR, 7, 8); // PWM 2 = Pin 7, DIR 2 = Pin 8.
CytronMD motor2(PWM_DIR, 6, 3); // PWM 1 = Pin 3, DIR 1 = Pin 6.

// Define the required yaw and pitch angles
//int requiredYawAngle = 20;
//int requiredPitchAngle = 25;
bool yawCalibrated = false;
bool pitchCalibrated = false;
int tolerance = 2; // Set your desired tolerance level

//==============================================================================================================
double latitude = 26.738180853550922;  // Jorhat latitude
double longitude = 94.21081269176345;  // Jorhat longitude
double utc_offset = 5.5;  // UTC offset for Jorhat (IST)
// Refresh interval, in seconds
int interval = 1;
//================================================================================================

void setup() {
  Serial.begin(19200);
  
  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
  setTime(toUtc(compileTime()));
}

void loop() {
  mpu6050.update();
  //===================
  static unsigned long next_millis = 0;
  // At every interval
  if (millis() > next_millis)
  {
    time_t utc = now();
    double az, el;

    // Calculate the solar position, in degrees
    calcHorizontalCoordinates(utc, latitude, longitude, az, el);

    //=============================
    //main conditional code to input here that will accept the az and el and move the motor
    if (!yawCalibrated) {
    // Yaw control
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
    // Pitch control
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
    
   //=============================

    next_millis = millis() + interval * 1000L;
  }
  //====================
  
}

//=================================================================
time_t toUtc(time_t local)
{
  return local - utc_offset * 3600L;
}

// Code from JChristensen/Timezone Clock example
time_t compileTime()
{
  const uint8_t COMPILE_TIME_DELAY = 8;
  const char *compDate = __DATE__, *compTime = __TIME__, *months = "JanFebMarAprMayJunJulAugSepOctNovDec";
  char chMon[4], *m;
  tmElements_t tm;

  strncpy(chMon, compDate, 3);
  chMon[3] = '\0';
  m = strstr(months, chMon);
  tm.Month = ((m - months) / 3 + 1);

  tm.Day = atoi(compDate + 4);
  tm.Year = atoi(compDate + 7) - 1970;
  tm.Hour = atoi(compTime);
  tm.Minute = atoi(compTime + 3);
  tm.Second = atoi(compTime + 6);
  time_t t = makeTime(tm);
  return t + COMPILE_TIME_DELAY;
}
