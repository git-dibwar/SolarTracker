#include "WMPU.h"
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
	#include "Wire.h"
#endif

WMPU::WMPU(IeepromMPU& ieepromMPU)
	:eepromMngr(ieepromMPU)
{

}

namespace nsMPU {
	MPU6050 mpu;
}

int WMPU::begin()
{
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
	Wire.begin();
	Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
	Fastwire::setup(400, true);
#endif
	nsMPU::mpu.initialize();
	nsMPU::mpu.testConnection();
	uint8_t devStatus;
	devStatus = nsMPU::mpu.dmpInitialize();
	guessOffsets();
	return devStatus;
}

void WMPU::Init() {
	uint8_t devStatus = begin();

	if (devStatus == 0) {
		if (isCalibrated()) {
			loadCalibration();
			nsMPU::mpu.setDMPEnabled(true);
		}
		else {
			Serial.println("MPU no calibrated. Halting execution.");
			while (true) {
				//failure: no calibration
			}
		}
	}
	else {
		Serial.println("Bad devStatus. Halting execution.");
		while (true) {
			//failure
		}
	}
}

void WMPU::getYawPitchRoll(float& y, float& p, float& r) {
	uint8_t fifoBuffer[64];
	Quaternion q;
	if (nsMPU::mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
		nsMPU::mpu.dmpGetQuaternion(&q, fifoBuffer);

		float q0 = q.w;
		float q1 = q.x;
		float q2 = q.y;
		float q3 = q.z;

		float yr = -atan2(-2 * q1 * q2 + 2 * q0 * q3, q2 * q2 - q3 * q3 - q1 * q1 + q0 * q0);
		float pr = asin(2 * q2 * q3 + 2 * q0 * q1);
		float rr = atan2(-2 * q1 * q3 + 2 * q0 * q2, q3 * q3 - q2 * q2 - q1 * q1 + q0 * q0);

		y = yr * 180 / M_PI;
		p = pr * 180 / M_PI;
		r = rr * 180 / M_PI;

	}
}

void WMPU::guessOffsets() {
	nsMPU::mpu.setXGyroOffset(220);
	nsMPU::mpu.setYGyroOffset(76);
	nsMPU::mpu.setZGyroOffset(-85);
	nsMPU::mpu.setZAccelOffset(1788);
}

void WMPU::loadCalibration()
{
	nsMPU::mpu.setXGyroOffset(eepromMngr.getXGyroOffset());
	nsMPU::mpu.setYGyroOffset(eepromMngr.getYGyroOffset());
	nsMPU::mpu.setZGyroOffset(eepromMngr.getZGyroOffset());

	nsMPU::mpu.setXAccelOffset(eepromMngr.getXAccelOffset());
	nsMPU::mpu.setYAccelOffset(eepromMngr.getYAccelOffset());
	nsMPU::mpu.setZAccelOffset(eepromMngr.getZAccelOffset());
}



void WMPU::Calibrate()
{
	Serial.println("Assuming module is flat and still!!!!!");
	uint8_t devStatus = begin();

	if (devStatus == 0) {
		//The following two command perform calibration and set obtained and set obtained affsets
		//as active offsets ( the ones DMP will use to generate quaternion)
		nsMPU::mpu.CalibrateAccel(6);
		nsMPU::mpu.CalibrateGyro(6);

		eepromMngr.setXGyroOffset(nsMPU::mpu.getXGyroOffset());
		eepromMngr.setYGyroOffset(nsMPU::mpu.getYGyroOffset());
		eepromMngr.setZGyroOffset(nsMPU::mpu.getZGyroOffset());

		eepromMngr.setXAccelOffset(nsMPU::mpu.getXAccelOffset());
		eepromMngr.setYAccelOffset(nsMPU::mpu.getYAccelOffset());
		eepromMngr.setZAccelOffset(nsMPU::mpu.getZAccelOffset());

		eepromMngr.setCalibFlag();
		Serial.println("Calibratrion complete.");

	}
	else {
		Serial.println("Bad devStatus. Halting execution.");
		while (true) {
			//failure
		}
	}
}

bool WMPU::isCalibrated()
{
	return eepromMngr.getCalibFlag();
}

void WMPU::resetCalibFlag()
{
	eepromMngr.resetCalibFlag();
	Serial.println("Calibratrion flag reset.");
}
