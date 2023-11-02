#include <MPU6050_tockn.h>
#include <DS1307RTC.h>
#include <Wire.h>

const char *monthName[12] = {
  "Jan", "Feb", "Mar", "Apr", "May", "Jun",
  "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"
};

MPU6050 mpu6050(Wire);
// DS1307RTC rtc;
tmElements_t tm;


void setup() {
  Serial.begin(19200);
  // put your setup code here, to run once:
  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
  bool parse = false;
  bool config = false;
  // get the date and time the compiler was run
  if (getDate(__DATE__) && getTime(__TIME__)) {
    parse = true;
    // and configure the RTC with this info
    if (RTC.write(tm)) {
      config = true;
    }
  }

  // Serial.begin(9600);
  while (!Serial);  // wait for Arduino Serial Monitor
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
}

void loop() {
  // put your main code here, to run repeatedly:
  mpu6050.update();
  Serial.print("DS1307 configured Time= "+String(__TIME__)+"   ,Date= "+String(__DATE__)) ;
  Serial.println();

  Serial.println("YawAngle: " + String(round(-mpu6050.getAngleZ())) + "     PitchAngle: " + String(round(mpu6050.getAngleY())));
  delay(2000);
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


