// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
//#include "MPU6050_tockn.h"
#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file
#include "CytronMotorDriver.h"
// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu (0x69);

#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards

//MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n' };

//INTERRUPT DETECTION ROUTINE
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}

CytronMD motor1(PWM_DIR, 6, 3);  // PWM 1 = Pin 3, DIR 1 = Pin 6.
CytronMD motor2(PWM_DIR, 7, 8); // PWM 2 = Pin 7, DIR 2 = Pin 8.


#define OUTPUT_READABLE_YAWPITCHROLL

//Define the required pitch and yaw angles
// Define the required pitch and yaw angles
double requiredPitchAngle = 25.0;
double requiredYawAngle = 20.0;


void setup() {
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  // initialize serial communication
  // (115200 chosen because it is required for Teapot Demo output, but it's
  // really up to you depending on your project)
  Serial.begin(115200);
  while (!Serial); // wait for Leonardo enumeration, others continue immediately

  // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3V or Arduino
  // Pro Mini running at 3.3V, cannot handle this baud rate reliably due to
  // the baud timing being too misaligned with processor ticks. You must use
  // 38400 or slower in these cases, or use some kind of external separate
  // crystal solution for the UART timer.

  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // wait for ready
  Serial.println(F("\nSend any character to begin DMP programming and demo: "));
  while (Serial.available() && Serial.read()); // empty buffer
  while (!Serial.available());                 // wait for data
  while (Serial.available() && Serial.read()); // empty buffer again

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.PrintActiveOffsets();
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
    Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
    Serial.println(F(")..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
  motor1.setSpeed(0); // Initialize motor speed to 0
  motor2.setSpeed(0);
}
bool pitchCalibrated = false;
bool yawCalibrated = false;
float tolerance = 1.00; // Set the tolerance value to 1 degree or adjust it as needed
//int pitchMotorPin = 9; // Replace with the actual pin number you are using
//int yawMotorPin = 10; // Replace with the actual pin number you are using

void loop() {
  if (!dmpReady) {
    return;
  }
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
#ifdef OUTPUT_READABLE_YAWPITCHROLL
    // display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    //            Serial.print("ypr\t");

    float yaw = ypr[0] * 180 / M_PI;
    float pitch = ypr[1] * 180 / M_PI;


    if (!pitchCalibrated) {
      // Calculate the pitch error
      float pitchError = requiredPitchAngle - pitch;
      float pitchMotorSpeed = pitchError * 10.0;

      // Set the motor direction for pitch
      if (pitchError > 0) {
        motor1.setSpeed(-128);
//        delay(1000);
      } else {
        motor1.setSpeed(128);
//        delay(1000);
      }

      // Control pitch motor speed
      float speedFactor = 1; // Adjust this factor to decrease the speed (0.5 means 50% speed)
      motor1.setSpeed(abs(pitchMotorSpeed) * speedFactor);


      // Check if pitch calibration is complete
      if (abs(pitchError) < tolerance) {
        // Pitch calibration is complete
        motor1.setSpeed(0);
        pitchCalibrated = true;
      }
    } else if (!yawCalibrated) {
      // Calculate the yaw error
      float yawError = requiredYawAngle - yaw;
      float yawMotorSpeed = yawError * 10.0;

      // Set the motor direction for yaw
      if (yawError < 0) {
        motor2.setSpeed(128);
//        delay(1000);
      } else {
        motor2.setSpeed(-128);
//        delay(1000);
      }

      // Control yaw motor speed
motor2.setSpeed(abs(yawMotorSpeed));

      // Check if yaw calibration is complete
      if (abs(yawError) < tolerance) {
        // Yaw calibration is complete
        motor2.setSpeed(0);
        yawCalibrated = true;
      }
    }

    Serial.print("yaw: " + String(yaw));
    Serial.print("\t");
    Serial.print("pitch: " + String(pitch));
    Serial.println();

#endif
  }


  delay(1000);
}
