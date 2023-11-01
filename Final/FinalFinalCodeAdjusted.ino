#include <MPU6050_tockn.h>
#include <Wire.h>
#include "CytronMotorDriver.h"
#include <SolarCalculator.h>
#include <TimeLib.h>
#include <DS1307RTC.h>

MPU6050 mpu6050(Wire);
CytronMD motor1(PWM_DIR, 7, 8); //yaw, pwm;dir
CytronMD motor2(PWM_DIR, 6, 3); //pitch, pwm;dir

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


const char *monthName[12] = {
  "Jan", "Feb", "Mar", "Apr", "May", "Jun",
  "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"
};

tmElements_t tm;

void setup() {
  Serial.begin(19200);
  bool parse=false;
  bool config=false;

  // get the date and time the compiler was run
  if (getDate(__DATE__) && getTime(__TIME__)) {
    parse = true;
    // and configure the RTC with this info
    if (RTC.write(tm)) {
      config = true;
    }
  }
  while (!Serial) ; // wait for Arduino Serial Monitor
  delay(200);
  if (parse && config) {
    Serial.print("DS1307 configured Time=");
    Serial.print(__TIME__);
    Serial.print(", Date=");
    Serial.println(__DATE__);
  } else if (parse) {
    Serial.println("DS1307 Communication Error :-{");
    Serial.println("Please check your circuitry");
  } else {
    Serial.print("Could not parse info from the compiler, Time=\"");
    Serial.print(__TIME__);
    Serial.print("\", Date=\"");
    Serial.print(__DATE__);
    Serial.println("\"");
  }
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
  int adjustmentAngle=0;

  if (!yawCalibrated) {
    int yaw = round(int(-mpu6050.getAngleZ()));
    int yawError =  -round(az) + yaw;

    if (abs(yawError) < tolerance) {
      motor1.setSpeed(0);
      yawCalibrated = true;
      Serial.println("Yaw calibration completed.");
    } else {
      if (yawError > 0) {
        motor1.setSpeed(128);
      } else {
        motor1.setSpeed(-128);
      }
    }
    Serial.println("Azimuth Angle: " + String(round(az))+"          Yaw Angle: "+String(round(yaw)));
    // Serial.println(az);
  } else if (!pitchCalibrated) {
    int pitch = round(int(mpu6050.getAngleY()));
    int pitchError = round(el + adjustmentAngle) - int(pitch);

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
    Serial.println("Elevation Angle: " + String(round(el))+"     Pitch Angle: "+String(round(pitch-adjustmentAngle)));
    // Serial.println(round(el));
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

bool getTime(const char *str)
{
  int Hour, Min, Sec;

  if (sscanf(str, "%d:%d:%d", &Hour, &Min, &Sec) != 3) return false;
  tm.Hour = Hour;
  tm.Minute = Min;
  tm.Second = Sec;
  return true;
}

bool getDate(const char *str)
{
  char Month[12];
  int Day, Year;
  uint8_t monthIndex;

  if (sscanf(str, "%s %d %d", Month, &Day, &Year) != 3) return false;
  for (monthIndex = 0; monthIndex < 12; monthIndex++) {
    if (strcmp(Month, monthName[monthIndex]) == 0) break;
  }
  if (monthIndex >= 12) return false;
  tm.Day = Day;
  tm.Month = monthIndex + 1;
  tm.Year = CalendarYrToTm(Year);
  return true;
}

